// SVS INCLUDES
#include "visual_operation.h"
#include "visual_sensory_memory.h"
#include "visual_operation_graph.h"


visual_operation_graph::visual_operation_graph(visual_sensory_memory* _vsm) {
    vsm = _vsm;
    node_images = *new int_image_map;
}

visual_operation_graph::~visual_operation_graph() {
    remove(0);
}

//////////////////////
// CLIPROXY METHODS //
//////////////////////
void visual_operation_graph::proxy_get_children(std::map<std::string, cliproxy*>& c) {
}

void visual_operation_graph::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VSM INTERFACE ==========" << std::endl;
    os << "======================================" << std::endl;
}

int visual_operation_graph::insert(std::unordered_map<std::string, int> parents, data_dict* params, void (*operation)(data_dict data_in)) {
    visual_operation_node* new_node;
    std::string parent_arg;
    int parent_id;
    visual_operation_node* parent_node;

    // Check if the parents actually exist. If one doesn't, return without inserting
    std::unordered_map<std::string, int>::iterator parents_itr;
    for (parents_itr=parents.begin(); parents_itr!=parents.end(); parents_itr++) {
        if (nodes.find(parents_itr->second) == nodes.end()) {
            return -1;
        }
    }

    new_node = new visual_operation_node;
    new_node->id = next_op_id;
    new_node->v_op = operation;
    new_node->parameters = *params;

    leaf_nodes.insert(new_node->id);  // A newly created node is always a leaf node    

    // Connect new child to parents and remove parents from leaf node set
    for (parents_itr=parents.begin(); parents_itr!=parents.end(); parents_itr++) {
        parent_arg = parents_itr->first;
        parent_id = parents_itr->second;
        parent_node = nodes[parent_id];

        parent_node->children.insert(new_node->id);
        new_node->parents[parent_arg] = parent_id;
        int parent_erased = leaf_nodes.erase(parent_id);  // Any parent is no longer a leaf node
    }
    
    nodes[new_node->id] = new_node;
    num_operations++;
    next_op_id++;
    return new_node->id;
}

int visual_operation_graph::remove(int target_id) {
    visual_operation_node* child;
    visual_operation_node* parent;
    visual_operation_node* target_node;

    if (nodes.find(target_id) == nodes.end()) {
        return num_operations;
    }
    target_node = nodes[target_id];

    // Remove all children of the target node
    std::unordered_set<int>::iterator children_itr;
    for (children_itr=target_node->children.begin(); children_itr != target_node->children.end(); children_itr++) {
        child = nodes[*children_itr];
        remove(child->id);
    }

    // Remove target node from all of its parents, and check them for leaf status
    std::unordered_map<std::string, int>::iterator parents_itr;
    for (parents_itr = target_node->parents.begin(); parents_itr != target_node->parents.end(); parents_itr++) {
            parent = nodes[parents_itr->second];
            parent->children.erase(target_id);
            if (parent->children.size() == 0) {
                leaf_nodes.insert(parent->id);
            }
        }
    
    delete &target_node;
    num_operations--;
    return num_operations;
}

void visual_operation_graph::evaluate() {
    visual_operation_node* leaf;

    std::unordered_set<int>::iterator leaf_itr;
    for (leaf_itr=leaf_nodes.begin(); 
        leaf_itr!=leaf_nodes.end(); 
        leaf_itr++) {
            leaf = nodes[*leaf_itr];
            evaluate_node(leaf);
    }

    // Clean up images in memory
    int_image_map::iterator node_images_itr;
    for (node_images_itr=node_images.begin(); 
        node_images_itr!=node_images.end(); 
        node_images_itr++) {
            delete node_images_itr->second;
    }
    node_images.clear();
}

void visual_operation_graph::evaluate_node(visual_operation_node* node) {
    visual_operation_node* parent;
    std::string parent_arg;
    opencv_image* parent_image_copy;

    // Get image arguments from the parents, if there are any
    std::unordered_map<std::string, int>::iterator parents_itr;
    for (parents_itr = node->parents.begin();
        parents_itr != node->parents.end();
        parents_itr++) {
            parent_arg = parents_itr->first;
            parent = nodes[parents_itr->second];
            // If there's no image for the parent, we need to evaluate it first
            if (node_images.find(parent->id) == node_images.end()) {
                evaluate_node(parent);
            }
            parent_image_copy = new opencv_image;
            parent_image_copy->copy_from(node_images[parent->id]);
            node->parameters[parent_arg] = parent_image_copy;
    }

    // If there are no parents, this is an image source node and needs a new opencv image
    if (node->parents.size() == 0) {
        opencv_image* target_image = new opencv_image;
        node->parameters["target"] = target_image;
    }

    // Regardless of parents, add the target image to `node_images`
    node_images[node->id] = (opencv_image*)node->parameters["target"];

    node->v_op(node->parameters);
}