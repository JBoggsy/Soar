// C++ STD Libraries
#include <string>
#include <vector>
// SVS INCLUDES
#include "visual_operation.h"
#include "visual_operation_graph.h"
#include "visual_working_memory.h"

///////////////////////////
// VISUAL OPERATION NODE //
///////////////////////////
visual_operation_node::visual_operation_node(std::string op_type, data_dict* params, int vop_node_id,
                                             visual_working_memory* vwm, soar_interface* si, Symbol* node_link)
    : op_type_(op_type), parameters_(*params), id_(vop_node_id), vwm_(vwm), si_(si), node_link_(node_link)
{
    op_metadata_ = visual_ops::vops_param_table[op_type];
    operation_   = op_metadata_.vop_function;
    node_image_ = NULL;

    // Populate the WM link with the op name and node id
    op_name_sym_ = si_->make_sym(op_type_);
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
        if (parameters_[param_name] == NULL) { continue; }

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
            case visual_ops::NODE_ID_ARG:
                param_val_int = *(int*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_int);
                parent_ids_[param_name] = param_val_int;
                break;
            default:
                param_val_int = *(int*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_int);
                break;
        }
        param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
    }
}

visual_operation_node::~visual_operation_node() {
    si_->del_sym(node_link_);
}

/**
 * @brief Edit the value of a parameter used as an arguments for the VOp in this node.
 *
 * @todo Handle changing a parent node argument properly, currently it doesn't update related nodes
 *
 * @param param_name The name of the parameter to edit
 * @param new_value The new value to give the parameter
 * @return true If the parameter is successfully edited
 * @return false If the parameter is not changed
 */
bool visual_operation_node::edit_parameter(std::string param_name, int new_value) {
    std::vector<std::string>::iterator param_names_start = op_metadata_.param_names.begin();
    std::vector<std::string>::iterator param_names_end = op_metadata_.param_names.end();
    std::vector<std::string>::iterator param_name_itr;
    param_name_itr = std::find(param_names_start, param_names_end, param_name);
    if (param_name_itr == param_names_end) {return false; }
    int param_index = std::distance(param_names_start, param_name_itr);

    visual_ops::ArgType param_type = op_metadata_.param_types.at(param_index);
    if ( (param_type != visual_ops::INT_ARG) || (param_type != visual_ops::NODE_ID_ARG) ) { return false; }

    *((int*)parameters_[param_name]) = new_value;
    si_->del_sym(param_syms_[param_name]);
    si_->remove_wme(param_wmes_[param_name]);
    param_syms_[param_name] = si_->make_sym(new_value);
    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);

    return true;
}
/**
 * @brief Edit the value of a parameter used as an arguments for the VOp in this node.
 *
 * @param param_name The name of the parameter to edit
 * @param new_value The new value to give the parameter
 * @return true If the parameter is successfully edited
 * @return false If the parameter is not changed
 */
bool visual_operation_node::edit_parameter(std::string param_name, double new_value) {
    std::vector<std::string>::iterator param_names_start = op_metadata_.param_names.begin();
    std::vector<std::string>::iterator param_names_end = op_metadata_.param_names.end();
    std::vector<std::string>::iterator param_name_itr;
    param_name_itr = std::find(param_names_start, param_names_end, param_name);
    if (param_name_itr == param_names_end) {return false; }
    int param_index = std::distance(param_names_start, param_name_itr);

    visual_ops::ArgType param_type = op_metadata_.param_types.at(param_index);
    if ( param_type != visual_ops::DOUBLE_ARG ) { return false; }

    *((double*)parameters_[param_name]) = new_value;
    si_->del_sym(param_syms_[param_name]);
    si_->remove_wme(param_wmes_[param_name]);
    param_syms_[param_name] = si_->make_sym(new_value);
    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
    return true;
}
/**
 * @brief Edit the value of a parameter used as an arguments for the VOp in this node.
 *
 * @param param_name The name of the parameter to edit
 * @param new_value The new value to give the parameter
 * @return true If the parameter is successfully edited
 * @return false If the parameter is not changed
 */
bool visual_operation_node::edit_parameter(std::string param_name, std::string new_value) {
    std::vector<std::string>::iterator param_names_start = op_metadata_.param_names.begin();
    std::vector<std::string>::iterator param_names_end = op_metadata_.param_names.end();
    std::vector<std::string>::iterator param_name_itr;
    param_name_itr = std::find(param_names_start, param_names_end, param_name);
    if (param_name_itr == param_names_end) {return false; }
    int param_index = std::distance(param_names_start, param_name_itr);

    visual_ops::ArgType param_type = op_metadata_.param_types.at(param_index);
    if ( param_type != visual_ops::STRING_ARG ) { return false; }

    *((std::string*)parameters_[param_name]) = new_value;
    si_->del_sym(param_syms_[param_name]);
    si_->remove_wme(param_wmes_[param_name]);
    param_syms_[param_name] = si_->make_sym(new_value);
    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
    return true;
}

/**
 * @brief Evaluate the visual operation node.
 *
 * @invariant `NODE_ID_ARG` type params will be NULL when evaluate is called,
 *            meaning this method doesn't need to `delete` old `opencv_image`
 *            instances.
 * @invariant the creation of new `opencv_image` instances for `NODE_ID_ARG` type
 *            params is handled by the `vog_->get_node_image` call, meaning
 *            this method doesn't need to `new` any opencv_image instances.
 *
 * @returns True if the evaluation was successful, false otherwise.
 */
bool visual_operation_node::evaluate() {
    // printf("Evaluating node %d...\n", id_);
    std::string parent_param_name;
    int parent_node_id;
    std::unordered_map<std::string, int>::iterator parent_itr;
    for (parent_itr=parent_ids_.begin(); parent_itr!=parent_ids_.end(); parent_itr++) {
        parent_param_name = parent_itr->first;
        parent_node_id = parent_itr->second;

        parameters_[parent_param_name] = vwm_->get_node_image(parent_node_id);
        if (parameters_[parent_param_name] == NULL) { printf("ERROR: Node %d not found\n", parent_node_id); }
    }
    operation_(parameters_);

    // Update WM outputs
    std::string param_name;
    visual_ops::ArgType param_type;
    visual_ops::ArgDirection param_dir;
    int         param_val_int;
    double      param_val_dbl;
    std::string param_val_str;
    for (int param_i=0; param_i<op_metadata_.num_params; param_i++) {
        param_name = op_metadata_.param_names[param_i];
        param_type = op_metadata_.param_types[param_i];
        param_dir = op_metadata_.param_direction[param_i];
        if (parameters_[param_name] == NULL) { continue; }
        if (param_dir == visual_ops::INPUT_ARG || param_type == visual_ops::NODE_ID_ARG) { continue; }

        // si_->del_sym(param_syms_[param_name]);
        si_->remove_wme(param_wmes_[param_name]);

        switch (param_type) {
            case visual_ops::INT_ARG:
                param_val_int = *(int*)(parameters_[param_name]);
                param_syms_[param_name] = si_->make_sym(param_val_int);
                break;
            case visual_ops::DOUBLE_ARG:
                param_val_dbl = *(double*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_dbl);
                printf("Param %s: %f\n", param_name.c_str(), param_val_dbl);
                break;
            case visual_ops::STRING_ARG:
                param_val_str = *(std::string*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_str);
                break;
        }
        param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
    }

    // printf("Done with node %d\n", id_);

    if (op_type_.compare(VOP_SAVE_TO_FILE) != 0) {
        char debug_save_filename[64];
        snprintf(debug_save_filename, 64, "node-%d.json", id_);
        ((opencv_image*)parameters_["source"])->save_image_data(debug_save_filename);
    }

    return true;
}

opencv_image* visual_operation_node::get_node_image() {
    return (opencv_image*)parameters_["source"];
}

std::string visual_operation_node::get_dot_string() {
    std::string ret_str = std::string();
    ret_str.append("vop_");
    ret_str.append(std::to_string(id_));
    ret_str.append(" [label=\"]");
    ret_str.append(op_type_);
    ret_str.append(" \"];\n");

    std::unordered_set<int>::iterator child_ids_itr = child_ids_.begin();
    for (; child_ids_itr!=child_ids_.end(); child_ids_itr++) {
        ret_str.append("vop_");
        ret_str.append(std::to_string(id_));
        ret_str.append(" -> vop_");
        ret_str.append(std::to_string(*child_ids_itr));
        ret_str.append(";\n");
    }

    return ret_str;
}
