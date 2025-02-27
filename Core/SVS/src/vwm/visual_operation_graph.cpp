// C++ STD Libraries
#include <string>
#include <vector>
// SVS INCLUDES
#include "visual_operation.h"
#include "visual_operation_graph.h"
#include "visual_working_memory.h"
#include "object_representation.h"

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


    // Populate parent_ids_, parent_types_, and the WM link by scanning over op_metadata_
    std::string param_name;
    visual_ops::ArgType param_type;
    visual_ops::ArgDirection param_dir;

    int         param_val_int;
    double      param_val_dbl;
    std::string param_val_str;

    opencv_image* param_val_img;
    wme* empty_wme;
    wme* img_width_wme;
    wme* img_height_wme;

    std::vector<opencv_image*>* param_val_img_vec;
    wme* num_images_wme;

    OBJ_REP_TYPE* param_val_obj;
    wme* obj_num_sides_wme;
    wme* obj_num_corners_wme;
    wme* obj_ellipsity_wme;

    #ifdef ENABLE_TORCH
    latent_representation* param_val_latent;
    wme* latent_size_wme;
    #endif

    for (int param_i=0; param_i<op_metadata_.num_params; param_i++) {
        param_name = op_metadata_.param_names[param_i];
        param_type = op_metadata_.param_types[param_i];
        param_dir = op_metadata_.param_direction[param_i];
        if (parameters_[param_name] == NULL) { continue; }

        switch (param_type) {
            case visual_ops::INT_ARG:
                param_val_int = *(int*)(parameters_[param_name]);
                param_syms_[param_name] = si_->make_sym(param_val_int);
                param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                break;
            case visual_ops::DOUBLE_ARG:
                param_val_dbl = *(double*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_dbl);
                param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                break;
            case visual_ops::STRING_ARG:
                param_val_str = *(std::string*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_str);
                param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                break;
            case visual_ops::CV_IMAGE_ARG:
                if (param_dir != visual_ops::OUTPUT_ARG) {
                    param_val_int = *(int*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(param_val_int);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                    parent_ids_[param_name] = param_val_int;
                    parent_types_[param_name] = param_type;
                } else {
                    param_val_img = (opencv_image*)parameters_[param_name];
                    param_wmes_[param_name] = si_->make_id_wme(node_link_, param_name);
                    bool empty = param_val_img->is_empty();
                    empty_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("empty"), si_->make_sym(empty));

                    if (!empty) {
                        img_width_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("width"), si_->make_sym(param_val_img->get_width()));
                        img_height_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("height"), si_->make_sym(param_val_img->get_height()));
                    }
                }
                break;
            case visual_ops::MULTI_CV_IMAGE_ARG:
                if (param_dir != visual_ops::OUTPUT_ARG) {
                    param_val_int = *(int*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(param_val_int);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                    parent_ids_[param_name] = param_val_int;
                    parent_types_[param_name] = param_type;
                } else {
                    param_val_img_vec = (std::vector<opencv_image*>*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(-1);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                }
            case visual_ops::LATENT_REP_ARG:
                #ifdef ENABLE_TORCH
                if (param_dir != visual_ops::OUTPUT_ARG) {
                    param_val_int = *(int*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(param_val_int);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                    parent_ids_[param_name] = param_val_int;
                    parent_types_[param_name] = param_type;
                } else {
                    param_val_latent = (latent_representation*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(-1);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                }
                #endif
                break;
            case visual_ops::OBJECT_ARG:
                if (param_dir != visual_ops::OUTPUT_ARG) {
                    param_val_int = *(int*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(param_val_int);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                    parent_ids_[param_name] = param_val_int;
                    parent_types_[param_name] = param_type;
                } else {
                    param_val_obj = (OBJ_REP_TYPE*)(parameters_[param_name]);
                    param_wmes_[param_name] = si_->make_id_wme(node_link_, param_name);
                    obj_num_sides_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("num-sides"), -1);
                    obj_num_corners_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("num-corners"), -1);
                    obj_ellipsity_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("ellipsity"), -1);
                }
                break;
            default:
                param_val_int = *(int*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_int);
                param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                break;
        }
    }
}

visual_operation_node::~visual_operation_node() {
    si_->del_sym(node_link_);
}

// visual_operation_node::create_obj_vec_param_wmes(std::string param_name, std::vector<OBJ_REP_TYPE*> obj_vec) {
//     if (op_metadata_.param_types[param_name] != visual_ops::OBJECT_VEC_ARG) { return; }

//     // Add WME for the number of objects
//     std::string num_objects_param_subname = param_name + std::string(".num-objects");
//     Symbol* num_objects_sym = si_->make_sym(obj_vec.size());
//     param_syms_[num_objects_param_subname] = num_objects_sym;
//     param_wmes_[num_objects_param_subname] = si_->make_wme(param_syms_[param_name], "num-objects", num_objects_sym);

//     // Add WMEs for each object
//     std::string object_param_subname;
//     std::vector<OBJ_REP_TYPE*>::iterator obj_vec_itr = obj_vec.begin();
//     for (; obj_vec_itr != obj_vec.end(); obj_vec_itr++) {
//         object_param_subname = param_name + std::string(".object")+std::to_string(std::distance(obj_vec.begin(), obj_vec_itr));
//         param_wmes_[object_param_subname] = si_->make_id_wme(param_syms_[param_name], "object");

//         // Add WME for the object id
//         param_syms_[object_param_subname + std::string("id")] = si_->make_sym(std::distance(obj_vec.begin(), obj_vec_itr))
//         param_wmes_[object_param_subname + std::string("id")] = si_->make_wme(param_wmes_[object_param_subname], "id", param_syms_[object_param_subname + std::string("id")]);

//         // Add WME for the object type
//         param_syms_[object_param_subname + std::string("type")] = si_->make_sym((*obj_vec_itr)->type);

//     }
// }

std::map<std::string, int> visual_operation_node::get_param_names_and_types() {
    std::map<std::string, int> params_info;
    int param_i;

    for (param_i=0; param_i<op_metadata_.num_params; param_i++) {
        if (parameters_[op_metadata_.param_names[param_i]] == NULL) { continue; }
        params_info[op_metadata_.param_names[param_i]] = (int)op_metadata_.param_types[param_i];
    }

    return params_info;
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
    if ( (param_type != visual_ops::INT_ARG) || (param_type != visual_ops::CV_IMAGE_ARG) ) { return false; }

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
 * @invariant `CV_IMAGE_ARG` type params will be NULL when evaluate is called,
 *            meaning this method doesn't need to `delete` old `opencv_image`
 *            instances.
 * @invariant the creation of new `opencv_image` instances for `CV_IMAGE_ARG`
 *            type params is handled by the `vog_->get_node_image` call, meaning
 *            this method doesn't need to `new` any opencv_image instances.
 *
 * @note This method assumes that all parent nodes have already been evaluated.
 * This is safe because each VOp node is evaluated as soon as it is created (in
 * `visual_working_memory::add_vop_node`), so there should be no un-evaluated
 * nodes in the VOG.
 *
 * @returns True if the evaluation was successful, false otherwise.
 */
bool visual_operation_node::evaluate() {
    // printf("Evaluating node %d...\n", id_);
    std::string parent_param_name;
    int parent_node_id;
    visual_ops::ArgType parent_param_type;
    std::unordered_map<std::string, int>::iterator parent_itr;
    for (parent_itr=parent_ids_.begin(); parent_itr!=parent_ids_.end(); parent_itr++) {
        parent_param_name = parent_itr->first;
        parent_node_id = parent_itr->second;
        parent_param_type = parent_types_[parent_param_name];

        switch (parent_param_type) {
            case visual_ops::CV_IMAGE_ARG:
                opencv_image* parent_image;
                parent_image = vwm_->get_node_image(parent_node_id, parent_param_name);
                parameters_[parent_param_name] = parent_image;
                break;
            case visual_ops::MULTI_CV_IMAGE_ARG:
                std::vector<opencv_image*>* parent_image_vec;
                parent_image_vec = vwm_->get_node_image_vec(parent_node_id, parent_param_name);
                parameters_[parent_param_name] = parent_image_vec;
                break;
            case visual_ops::LATENT_REP_ARG:
                #ifdef ENABLE_TORCH
                latent_representation* parent_latent_rep;
                parent_latent_rep = vwm_->get_node_latent_rep(parent_node_id, parent_param_name);
                parameters_[parent_param_name] = parent_latent_rep;
                #else
                opencv_image* latent_image;
                latent_image = vwm_->get_node_image(parent_node_id, parent_param_name);
                parameters_[parent_param_name] = latent_image;
                #endif
                break;
            case visual_ops::OBJECT_ARG:
                parameters_[parent_param_name] = vwm_->get_object_rep(parent_node_id);
                break;

        }
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

    opencv_image* param_val_img;
    wme* empty_wme;
    wme* img_width_wme;
    wme* img_height_wme;

    std::vector<opencv_image*>* param_val_img_vec;
    wme* num_images_wme;

    OBJ_REP_TYPE* param_val_obj;
    wme* obj_num_sides_wme;
    wme* obj_num_corners_wme;
    wme* obj_ellipsity_wme;

    #ifdef ENABLE_TORCH
    latent_representation* param_val_latent;
    wme* latent_size_wme;
    #endif

    for (int param_i=0; param_i<op_metadata_.num_params; param_i++) {
        param_name = op_metadata_.param_names[param_i];
        param_type = op_metadata_.param_types[param_i];
        param_dir = op_metadata_.param_direction[param_i];
        if (parameters_[param_name] == NULL) { continue; }
        if (param_dir == visual_ops::INPUT_ARG) { continue; }

        // si_->del_sym(param_syms_[param_name]);
        si_->remove_wme(param_wmes_[param_name]);

        switch (param_type) {
            case visual_ops::INT_ARG:
                param_val_int = *(int*)(parameters_[param_name]);
                param_syms_[param_name] = si_->make_sym(param_val_int);
                param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                break;
            case visual_ops::DOUBLE_ARG:
                param_val_dbl = *(double*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_dbl);
                param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                break;
            case visual_ops::STRING_ARG:
                param_val_str = *(std::string*)parameters_[param_name];
                param_syms_[param_name] = si_->make_sym(param_val_str);
                param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                break;
            case visual_ops::CV_IMAGE_ARG:
                if (param_dir != visual_ops::OUTPUT_ARG) {
                    param_val_int = *(int*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(param_val_int);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                    parent_ids_[param_name] = param_val_int;
                    parent_types_[param_name] = param_type;
                } else {
                    param_val_img = (opencv_image*)parameters_[param_name];
                    param_wmes_[param_name] = si_->make_id_wme(node_link_, param_name);
                    bool empty = param_val_img->is_empty();
                    empty_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("empty"), si_->make_sym(empty));

                    if (!empty) {
                        img_width_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("width"), si_->make_sym(param_val_img->get_width()));
                        img_height_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("height"), si_->make_sym(param_val_img->get_height()));
                    }
                }
                break;
            case visual_ops::MULTI_CV_IMAGE_ARG:
                if (param_dir != visual_ops::OUTPUT_ARG) {
                    param_val_int = *(int*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(param_val_int);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                    parent_ids_[param_name] = param_val_int;
                    parent_types_[param_name] = param_type;
                } else {
                    param_val_img_vec = (std::vector<opencv_image*>*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(-1);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                }
                break;
            case visual_ops::LATENT_REP_ARG:
                #ifdef ENABLE_TORCH
                if (param_dir != visual_ops::OUTPUT_ARG) {
                    param_val_int = *(int*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(param_val_int);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                    parent_ids_[param_name] = param_val_int;
                    parent_types_[param_name] = param_type;
                } else {
                    param_val_latent = (latent_representation*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(-1);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                }
                #endif
                break;
            case visual_ops::OBJECT_ARG:
                if (param_dir != visual_ops::OUTPUT_ARG) {
                    param_val_int = *(int*)parameters_[param_name];
                    param_syms_[param_name] = si_->make_sym(param_val_int);
                    param_wmes_[param_name] = si_->make_wme(node_link_, param_name, param_syms_[param_name]);
                    parent_ids_[param_name] = param_val_int;
                    parent_types_[param_name] = param_type;
                } else {
                    param_val_obj = (OBJ_REP_TYPE*)(parameters_[param_name]);
                    param_wmes_[param_name] = si_->make_id_wme(node_link_, param_name);
                    obj_num_sides_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("num-sides"), -1);
                    obj_num_corners_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("num-corners"), -1);
                    obj_ellipsity_wme = si_->make_wme(param_wmes_[param_name]->value, std::string("ellipsity"), -1);
                }
                break;
        }
    }

    // printf("Done with node %d\n", id_);

    if (op_type_.compare(VOP_SAVE_TO_FILE) != 0) {
        char debug_save_filename[64];
        snprintf(debug_save_filename, 64, "node-%d.json", id_);
        ((opencv_image*)parameters_["source"])->save_image_data(debug_save_filename);
    }

    return true;
}

opencv_image* visual_operation_node::get_node_image() { return get_node_image("source"); }
opencv_image* visual_operation_node::get_node_image(std::string param_name) {
    return (opencv_image*)parameters_[param_name];
}

std::vector<opencv_image*>* visual_operation_node::get_node_image_vec(std::string param_name) {
    return (std::vector<opencv_image*>*)parameters_[param_name];
}

#ifdef ENABLE_TORCH
latent_representation* visual_operation_node::get_node_latent_rep(std::string param_name) {
    return (latent_representation*)parameters_[param_name];
}
#endif

OBJ_REP_TYPE* visual_operation_node::get_object_rep() {
    return (OBJ_REP_TYPE*)parameters_[VOP_ARG_OBJECT];
}

std::string visual_operation_node::get_dot_string() {
    std::string ret_str = std::string();
    ret_str.append("vop_");
    ret_str.append(std::to_string(id_));
    ret_str.append(" [label=\"");
    ret_str.append(std::to_string(id_));
    ret_str.append(": ");
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
