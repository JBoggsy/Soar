#include <time.h>

#include "svs.h"
#include "visual_working_memory.h"
#include "image.h"
#include "symbol.h"
#include "soar_interface.h"
// Base64 library for image transfer
#include "base64.h"


#ifdef ENABLE_ROS
const std::string visual_working_memory::ROS_TOPIC_NAME = "vwm";
#endif

// CREATION/DESTRUCTION METHODS
///////////////////////////////
visual_working_memory::visual_working_memory(svs* svs_ptr, soar_interface* si, Symbol* vwm_link)
    : svs_ptr(svs_ptr), si(si), vwm_link(vwm_link)
{
    num_operations = 0;
    num_operations_symbol = si->make_sym(num_operations);
    num_operations_wme = si->make_wme(vwm_link, std::string("num_ops"), num_operations_symbol);
    next_vop_node_id = 0;

    #ifdef ENABLE_ROS
    svs_ptr->get_ros_interface()->create_rgb_publisher(ROS_TOPIC_NAME);
    #endif
}

visual_working_memory::~visual_working_memory() {
}

visual_working_memory* visual_working_memory::clone(Symbol* vwm_link) {
    visual_working_memory* new_vwm = new visual_working_memory(svs_ptr, si, vwm_link);
    return new_vwm;
}

// VOP GRAPH METHODS
////////////////////
int visual_working_memory::add_visual_operation(std::string op_type, data_dict* op_args, std::unordered_map<std::string, int> parent_ids) {
    visual_ops::vop_params_metadata op_metadata = visual_ops::vops_param_table[op_type];



    Symbol* new_node_link = si->get_wme_val(si->make_id_wme(vwm_link, std::string("node")));
    visual_operation_node* new_node = new visual_operation_node(op_type, op_args, assign_new_node_id(), this, si, new_node_link);
    // Now that the VOP is created and added, link it to its source nodes
    std::unordered_map<std::string, int>::iterator parents_itr;
    for (parents_itr=parent_ids.begin(); parents_itr!=parent_ids.end(); parents_itr++) {
        // If the parent id is -1, it means there's no parent specified, which is fine
        if (parents_itr->second == -1) {
            continue;
        }
        // If the parent doesn't exist, delete the new node and return failure value
        else if (vop_nodes.find(parents_itr->second) == vop_nodes.end()) {
            delete new_node;
            return -1;
        }
        // Otherwise, link the parent to the child
        else {
            add_child_to_node(new_node->get_id(), parents_itr->second);
        }
    }

    vop_nodes[new_node->get_id()] = new_node;
    num_operations++;

    return new_node->get_id();

}

int visual_working_memory::remove_visual_operation(int node_id) {
    visual_operation_node* child;
    visual_operation_node* parent;
    visual_operation_node* source_node;

    if (vop_nodes.find(node_id) == vop_nodes.end()) {
        return num_operations;
    }
    source_node = vop_nodes[node_id];

    // Remove all children of the source node
    std::unordered_set<int>::iterator children_itr;
    for (children_itr=source_node->get_child_ids()->begin(); children_itr != source_node->get_child_ids()->end(); children_itr++) {
        child = vop_nodes[*children_itr];
        remove_visual_operation(child->get_id());
    }

    // Remove source node from all of its parents, and check them for leaf status
    std::unordered_map<std::string, int>::iterator parents_itr;
    for (parents_itr = source_node->get_parent_ids()->begin(); parents_itr != source_node->get_parent_ids()->end(); parents_itr++) {
            parent = vop_nodes[parents_itr->second];
            parent->remove_child_id(node_id);
        }

    delete &source_node;
    num_operations--;
    return num_operations;

}

bool visual_working_memory::evaluate_node(int node_id) {
    visual_operation_node* target_node = vop_nodes[node_id];
    bool evaluation_parity = target_node->evaluate(evaluation_parity);

    std::unordered_set<int>::iterator child_node_ids_itr;
    child_node_ids_itr = target_node->get_child_ids()->begin();
    for (;child_node_ids_itr != target_node->get_child_ids()->end(); child_node_ids_itr++) {
        get_node(*child_node_ids_itr)->evaluate(evaluation_parity);
    }
}

bool visual_working_memory::add_child_to_node(int child_id, int parent_id) {
    if (vop_nodes.find(parent_id) == vop_nodes.end()) {
        return false;
    }

    vop_nodes[parent_id]->add_child_id(child_id);
    return true;
}

visual_operation_node* visual_working_memory::get_node(int node_id) {
    if (vop_nodes.find(node_id) == vop_nodes.end()) { return NULL; }
    return vop_nodes.at(node_id);
}

opencv_image* visual_working_memory::get_node_image(int node_id) {
    visual_operation_node* target_node = get_node(node_id);
    if (target_node == NULL) {
        return NULL;
    } else {
        return target_node->get_node_image();
    }
}

int visual_working_memory::assign_new_node_id() {
    return next_vop_node_id++;
}

std::string visual_working_memory::get_vog_dot_string() {
    std::string ret_str = std::string("digraph {\n");

    id_node_map::iterator nodes_itr = vop_nodes.begin();
    for (; nodes_itr!=vop_nodes.end(); nodes_itr++) {
        ret_str.append(nodes_itr->second->get_dot_string());
    }

    ret_str.append("}");
    return ret_str;
}

//////////////////////
// CLIPROXY METHODS //
//////////////////////
void visual_working_memory::proxy_get_children(std::map<std::string, cliproxy*>& c) {

}

void visual_working_memory::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {

}
