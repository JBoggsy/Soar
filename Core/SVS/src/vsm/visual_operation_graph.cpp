// SVS INCLUDES
#include "visual_operation.h"
#include "visual_sensory_memory.h"
#include "visual_operation_graph.h"

///////////////////////////
// VISUAL OPERATION NODE //
///////////////////////////
visual_operation_node::visual_operation_node(std::string op_name, data_dict* params, 
                                             visual_operation_graph* vog, soar_interface* si, Symbol* node_link) {
    op_name_     = op_name;
    op_metadata_ = visual_ops::vops_param_table[op_name];
    parameters_  = *params;
    operation_   = op_metadata_.vop_function;
    id_          = vog->assign_new_node_id();

    vog_ = vog;
    node_image_ = NULL;

    si_ = si;
    node_link_ = node_link;

    // Populate the WM link with the op name and node id
    op_name_sym_ = si_->make_sym(op_name_);
    si_->make_wme(node_link_, std::string("op-name"), op_name_sym_);

    node_id_sym_ = si_->make_sym(id_);
    si_->make_wme(node_link_, std::string("node-id"), node_id_sym_);

    // Populate parent_ids_ and the WM link by scanning over op_metadata_
    std::string param_name;
    visual_ops::ArgType param_type;
    int         param_val_int;
    double      param_val_dbl;
    std::string param_val_str;
    for (int param_i=0; param_i<op_metadata_.num_params; param_i++) {
        param_name = op_metadata_.param_names[param_i];
        param_type = op_metadata_.param_types[param_i];
        if (parameters_[param_name] == NULL) {
            continue;
        }

        switch (param_type) {
            case visual_ops::INT_ARG:
                param_val_int = *(int*)(parameters_[param_name]);
                param_syms_[param_name] = si_->make_sym(param_val_int);
                break;
            case visual_ops::DOUBLE_ARG:
                param_val_dbl = *(double*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_dbl);
                break;
            case visual_ops::STRING_ARG:
                param_val_str = *(std::string*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_str);
                break;
            case visual_ops::IMAGE_ARG:
                param_val_int = *(int*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_int);
                parent_ids_[param_name] = param_val_int;
                vog_->add_child_to_node(id_, param_val_int);
                break;
            default:
                param_val_int = *(int*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_int);
                break;
        }
        si_->make_wme(node_link_, param_name, param_syms_[param_name]);
    }
}

visual_operation_node::~visual_operation_node() {
    si_->del_sym(node_link_);
}

/**
 * @brief Evaluate the visual operation node
 * 
 * @invariant `IMAGE_ARG` type params will be NULL when evaluate is called,
 *            meaning this method doesn't need to `delete` old `opencv_image`
 *            instances.
 * @invariant the creation of new `opencv_image` instances for `IMAGE_ARG` type
 *            params is handled by the `vog_->get_node_image` call, meaning
 *            this method doesn't need to `new` any opencv_image instances.
 * 
 * @returns true iff evaluation succeeded
 */
bool visual_operation_node::evaluate() {
    std::string parent_param_name;
    int parent_node_id;
    std::unordered_map<std::string, int>::iterator parent_itr;
    for (parent_itr=parent_ids_.begin(); parent_itr!=parent_ids_.end(); parent_itr++) {
        parent_param_name = parent_itr->first;
        parent_node_id = parent_itr->second;

        parameters_[parent_param_name] = vog_->get_node_image(parent_node_id);
    }

    operation_(parameters_);
    vog_->mark_node_evaluated(id_);
    return true;
}

opencv_image* visual_operation_node::get_node_image() {
    return (opencv_image*)parameters_["target"];
}

////////////////////////////
// VISUAL OPERATION GRAPH //
////////////////////////////

visual_operation_graph::visual_operation_graph(visual_sensory_memory* vsm, soar_interface* si, Symbol* vsm_link) {
    vsm_ = vsm;
    si_ = si;
    
    num_operations_ = 0;
    next_node_id_ = 0;

    vog_link_ = si_->get_wme_val(si_->make_id_wme(vsm_link, "vog"));
    num_ops_sym_ = si_->get_wme_val(si->make_wme(vsm_link, "size", num_operations_));
}

visual_operation_graph::~visual_operation_graph() {
    remove(0);
}

/**
 * @brief Insert a new operation into the graph as a child of the nodes
 * with the specified ids. 
 * @param `op_name`: The name of the operation of the new node. This should
 *        correspond with a valid key in `visual_ops::vops_param_table`.
 * @param `params`: A newly allocated `data_dict` containing the 
 *        parameters for the visual operation
 * 
 * @returns The ID of the new visual operations in the graph. If insertion
 * fails, will return -1.
 */
int visual_operation_graph::insert(std::string op_name, data_dict* params, std::unordered_map<std::string, int> parent_ids) {
    visual_ops::vop_params_metadata op_metadata = visual_ops::vops_param_table[op_name];

    // Check if the parents actually exist; -1 means no parent and is a valid value!
    std::unordered_map<std::string, int>::iterator parents_itr;
    for (parents_itr=parent_ids.begin(); parents_itr!=parent_ids.end(); parents_itr++) {
        if (parents_itr->second == -1) {
            continue;
        }
        if (nodes_.find(parents_itr->second) == nodes_.end()) {
            return -1;
        }
    }
    
    Symbol* new_node_link = si_->get_wme_val(si_->make_id_wme(vog_link_, std::string("node")));
    visual_operation_node* new_node = new visual_operation_node(op_name, params, this, si_, new_node_link);
    nodes_[new_node->get_id()] = new_node;
    num_operations_++;
    add_leaf_node(new_node->get_id());  // A newly created node is always a leaf node

    return new_node->get_id();
}

/**
 * @brief Add the given child node to the given parent node, and ensure the parent
 * node is no longer marked as a leaf node.
 * 
 * @param child_id 
 * @param parent_id 
 * @return true The child node id was successfully added to the parent node.
 * @return false The child node id couldn't be added to the parent node.
 */
bool visual_operation_graph::add_child_to_node(int child_id, int parent_id) {
    if (nodes_.find(parent_id) == nodes_.end()) {
        return false;
    }

    nodes_[parent_id]->add_child_id(child_id);
    leaf_nodes_.erase(parent_id);

    return true;
}

/**
 * @brief Remove the operation with the specified id. 
 * ALL CHILDREN ARE DESTROYED. The root node cannot be removed.
 * 
 * @returns The new number of visual operations in the graph. If removal
 * fails, the number will be the same as it was prior to the operation.
 */
int visual_operation_graph::remove(int target_id) {
    visual_operation_node* child;
    visual_operation_node* parent;
    visual_operation_node* target_node;

    if (nodes_.find(target_id) == nodes_.end()) {
        return num_operations_;
    }
    target_node = nodes_[target_id];

    // Remove all children of the target node
    std::unordered_set<int>::iterator children_itr;
    for (children_itr=target_node->get_child_ids()->begin(); children_itr != target_node->get_child_ids()->end(); children_itr++) {
        child = nodes_[*children_itr];
        remove(child->get_id());
    }

    // Remove target node from all of its parents, and check them for leaf status
    std::unordered_map<std::string, int>::iterator parents_itr;
    for (parents_itr = target_node->get_parent_ids()->begin(); parents_itr != target_node->get_parent_ids()->end(); parents_itr++) {
            parent = nodes_[parents_itr->second];
            parent->remove_child_id(target_id);
            if (parent->get_child_ids()->size() == 0) {
                add_leaf_node(parents_itr->second);
            }
        }
    
    delete &target_node;
    num_operations_--;
    return num_operations_;
}

/**
 * @brief Designate the node with the specified ID as a leaf node(i.e.,
 * a node with no other nodes using it as a `target`.) This method does
 * NOT check that the given node is, in fact, a leaf node.
 * 
 * @param node_id The node id of the node to be designated a leaf node.
 */
void visual_operation_graph::add_leaf_node(int node_id) { 
    leaf_nodes_.insert(node_id);
}

/**
 * @brief Undesignate the node with the specified ID as a leaf node(i.e.,
 * a node with no other nodes using it as a `target`.) This method does
 * NOT check that the given node is not, in fact, a leaf node.
 * 
 * @param node_id The node id of the node to be undesignated as a leaf node.
 */
void visual_operation_graph::remove_leaf_node(int node_id) { 
    leaf_nodes_.erase(node_id); 
}

/**
 * @brief Evaluate every node in the graph once. Nodes are evaluated
 * recursively upwards, starting with the leaf nodes. In the process of
 * evaluating a node, its parent nodes are evaluated in order to generate
 * the image inputs for the node itself. Nodes are evaluated at most
 * once and the result of that evaluation is cached, to be accessed by
 * children of the node. Once every child of a node has been evaluated,
 * its image is destroyed to save memory.
 */
void visual_operation_graph::evaluate() {
    evaluated_nodes_.clear();

    visual_operation_node* leaf;
    std::unordered_set<int>::iterator leaf_itr;
    for (leaf_itr=leaf_nodes_.begin(); 
        leaf_itr!=leaf_nodes_.end(); 
        leaf_itr++) {
            leaf = nodes_[*leaf_itr];
            leaf->evaluate();
    }
}

/**
 * @brief Get the node image of the given node.
 * 
 * For now, assume a node ALWAYS has precisely 1 node image.
 * 
 * @param node_id 
 * @return opencv_image* The image of the given node, or NULL if such a
 *         node doesn't exist or cannot be evaluated.
 */
opencv_image* visual_operation_graph::get_node_image(int node_id) {
    if (nodes_.find(node_id) == nodes_.end()) {
        return NULL;
    }
    visual_operation_node* node = nodes_[node_id];

    if (evaluated_nodes_.find(node_id) == evaluated_nodes_.end()) {
        if (!node->evaluate()) {
            return NULL;
        }
    }

    opencv_image* ret_image;
    if (node->get_child_ids()->size() > 1) {
        ret_image = new opencv_image();
        ret_image->copy_from(node->get_node_image());
    } else {
        ret_image = node->get_node_image();
    }

    return ret_image;
}

/**
 * @brief Marks the node with the given id as evaluated for the current
 * evaluation run.
 */
void visual_operation_graph::mark_node_evaluated(int node_id) {
    evaluated_nodes_.insert(node_id);
}

// CLIPROXY METHODS
//////////////////////
void visual_operation_graph::proxy_get_children(std::map<std::string, cliproxy*>& c) {
}

void visual_operation_graph::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VSM INTERFACE ==========" << std::endl;
    os << "======================================" << std::endl;
}
