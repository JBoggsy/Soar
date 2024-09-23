#include <time.h>
// SVS includes
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

    #ifdef ENABLE_TORCH
    nn = new neural_network();
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
    if (op_type == VOP_GET_FROM_VIB) {
        std::string vib_id = *(std::string*)(op_args->at(VOP_ARG_VIBID));
        vib_vop_node_ids[vib_id].push_back(new_node->get_id());
    }
    num_operations++;

    evaluate_from_node(new_node->get_id());
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

bool visual_working_memory::evaluate_from_node(int node_id) {
    std::vector<int> frontier(1, node_id);
    int active_node_id;
    visual_operation_node* active_node;
    std::unordered_set<int>::iterator child_node_ids_itr;
    bool success;

    while (!frontier.empty()) {
        active_node_id = frontier.front();
        frontier.erase(frontier.begin());
        active_node = get_node(active_node_id);

        child_node_ids_itr = active_node->get_child_ids()->begin();
        for (;child_node_ids_itr != active_node->get_child_ids()->end(); child_node_ids_itr++) {
            frontier.push_back(*child_node_ids_itr);
        }

        success = active_node->evaluate();
        if (!success) {
            return false;
        }
    }

    return success;
}

bool visual_working_memory::evaluate_vib_nodes(std::string vib_id) {
    std::vector<int>::iterator vop_node_itr = vib_vop_node_ids[vib_id].begin();

    for (; vop_node_itr != vib_vop_node_ids[vib_id].end(); vop_node_itr++) {
        evaluate_from_node(*vop_node_itr);
    }

    return true;
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
    if (node_id == -1) { return new opencv_image(); }
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
    c["load-vae"] = new memfunc_proxy<visual_working_memory>(this, &visual_working_memory::cli_load_vae);
    c["load-vae"]->add_arg("VAE-FILE", "The path to the .pt file containing the traced VAE model to load.");

    c["nlist"] = new memfunc_proxy<visual_working_memory>(this, &visual_working_memory::cli_list_nodes);

    c["ninfo"] = new memfunc_proxy<visual_working_memory>(this, &visual_working_memory::cli_get_node_info);
    c["ninfo"]->add_arg("NID", "The id of the node to inspect.");

    c["nimg"] = new memfunc_proxy<visual_working_memory>(this, &visual_working_memory::cli_get_node_image);
    c["ninfo"]->add_arg("NID", "The id of the node whose image should be retrieved.");
    c["ninfo"]->add_arg("ARG", "The argument of the VOp whose image should be retrieved. Optional, defaults to 'source'.");

    c["vogdot"] = new memfunc_proxy<visual_working_memory>(this, &visual_working_memory::cli_get_vog_dot);
}

void visual_working_memory::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VISUAL WORKING MEMORY INTERFACE ==========" << std::endl;
    os << "NUMBER OF VOP NODES: " << num_operations << std::endl;
    os << "CLI USAGE:" << std::endl << std::endl;
    os << "svs vwm - Prints this help message."<< std::endl;
    os << "svs vwm.nlist - Prints a list of all of the VOp nodes in the VOG. Nodes are printed one per line, with the format <node-id> - <vop-type>."<< std::endl;
    os << "svs vwm.ninfo <NID> - Prints detailed information about the VOp node with the specified ID." << std::endl;
    os << "svs vwm.nimg <NID> <ARG-NAME> - Prints the image corresponding to the given argument of the VOp node with the specified ID. The image is printed as a base64-encoded .png. <ARG-NAME> is optional and defaults to 'source'." << std::endl;
    os << "svs vwm.vogdot - Recursively generates a DOT representation of the VOG and prints it. Used to display the VOG using a GraphViz renderer."<< std::endl;
    os << "========================================================" << std::endl;
}

void visual_working_memory::cli_load_vae(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "You must specify a path to the .pt file containing the VAE model." << std::endl;
        return;
    }

    std::string vae_file_path = args[0];
    os << "Loading VAE from " << vae_file_path << std::endl;

    // Load the VAE model
    nn->load_traced_script(vae_file_path);
}

void visual_working_memory::cli_list_nodes(const std::vector<std::string>& args, std::ostream& os) {
    os << "NID - VOP" << std::endl;

    id_node_map::iterator nodes_itr = vop_nodes.begin();
    id_node_map::iterator nodes_end = vop_nodes.end();
    for (; nodes_itr != nodes_end; nodes_itr++) {
        os << nodes_itr->first << " - " << nodes_itr->second->get_op_type().c_str() << std::endl;
    }
}
void visual_working_memory::cli_get_node_info(const std::vector<std::string>& args, std::ostream& os) {
    int node_id;
    visual_operation_node* vop_node;
    int i;
    std::map<std::string, int> params_info;
    int param_val_int;
    double param_val_dbl;
    std::string param_val_str;

    if (args.empty()) {
        os << "You must specify a node id." << std::endl;
        return;
    } else {
        node_id = std::stoi(args[0]);
    }

    vop_node = vop_nodes[node_id];
    os << "Node " << node_id << " (" << vop_node->get_op_type() << ")" << std::endl;
    os << "Children: ";
    i = 0;
    for (int child_id : *(vop_node->get_child_ids())) {
        os << child_id;
        if (i < (vop_node->get_child_ids()->size()-1)) {
            os << ",";
        }
        i++;
    }
    os << std::endl;
    os << "Parents: ";
    i = 0;
    for (std::pair<std::string, int> parent : *(vop_node->get_parent_ids())) {
        os << parent.second;
        if (i < (vop_node->get_parent_ids()->size()-1)) {
            os << ",";
        }
        i++;
    }
    os << std::endl;

    params_info = vop_node->get_param_names_and_types();
    for (std::pair<std::string, int> param_info : params_info) {
        os << param_info.first << ": ";
        if (param_info.second == 0) {
            os << vop_node->get_int_parameter(param_info.first);
        } else if (param_info.second == 1) {
            os << vop_node->get_dbl_parameter(param_info.first);
        } else if (param_info.second == 2) {
            os << vop_node->get_str_parameter(param_info.first);
        } else if (param_info.second == 3) {
            os << vop_node->get_parent_ids()->at(param_info.first);
        } else {
            os << "MEMORY ARG";
        }
        os << std::endl;
    }
}

void visual_working_memory::cli_get_node_image(const std::vector<std::string>& args, std::ostream& os) {
    int node_id;
    std::string arg_name;
    visual_operation_node* vop_node;
    opencv_image* node_img;
    cv::Mat* node_img_mat;
    std::vector<uchar> raw_png_data;
    std::string b64_data;

    if (args.empty()) {
        os << "You must specify a node id." << std::endl;
        return;
    } else {
        node_id = std::stoi(args[0]);
    }

    if (args.size() > 1) {
        arg_name = args[1];
    } else {
        arg_name = std::string("source");
    }

    vop_node = vop_nodes[node_id];
    node_img = vop_node->get_node_image();
    node_img_mat = node_img->get_image();
    cv::imencode(std::string(".png"), *(node_img_mat), raw_png_data);
    b64_data = base64_encode(raw_png_data.data(), raw_png_data.size());
    os << b64_data << std::endl;
}

void visual_working_memory::cli_get_vog_dot(const std::vector<std::string>& args, std::ostream& os) {
    os << get_vog_dot_string() << std::endl;
}
