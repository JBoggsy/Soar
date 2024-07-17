#ifndef VISUAL_OPERATION_GRAPH_H
#define VISUAL_OPERATION_GRAPH_H

//C++ STDLIB includes
#include <vector>
#include <unordered_map>
#include <unordered_set>
// SVS includes
#include "cliproxy.h"
#include "visual_operation_data_structs.h"
#include "visual_operation.h"
#include "image.h"
class visual_working_memory;


class visual_operation_node {
private:
    int id_;
    std::unordered_set<int> child_ids_;
    std::unordered_map<std::string, int> parent_ids_;

    std::string op_type_;
    data_dict parameters_;
    void (*operation_)(data_dict args);
    visual_ops::vop_params_metadata op_metadata_;

    visual_working_memory* vwm_;
    opencv_image* node_image_;
    bool evaluation_parity_;

    soar_interface* si_;
    Symbol* node_link_;
    Symbol* op_name_sym_;
    Symbol* node_id_sym_;
    std::unordered_map<std::string, Symbol*> param_syms_;
    std::unordered_map<std::string, wme*> param_wmes_;
public:
    visual_operation_node(std::string op_name, data_dict* params, int vop_node_id,
                          visual_working_memory* vwm, soar_interface* si, Symbol* node_link);
    ~visual_operation_node();

    std::string get_op_type() { return op_type_; }

    void set_id(int id) { id_ = id; }
    int get_id() { return id_; }
    std::unordered_set<int>* get_child_ids() { return &child_ids_; }
    std::unordered_map<std::string, int>* get_parent_ids() { return &parent_ids_; }
    bool get_evaluation_parity() { return evaluation_parity_; }
    bool flip_evaluation_parity() { evaluation_parity_ = !evaluation_parity_; }

    bool edit_parameter(std::string param_name, int new_value);
    bool edit_parameter(std::string param_name, double new_value);
    bool edit_parameter(std::string param_name, std::string new_value);

    void add_child_id(int id) { child_ids_.insert(id); }
    void remove_child_id(int id) {child_ids_.erase(id); }

    /**
     * @brief Evaluate the visual operation node, thereby updating its outputs.
     *
     * @return The value of the evaluation parity bit.
     */
    bool evaluate();
    bool evaluate(bool evaluation_parity);
    opencv_image* get_node_image();

    /**
     * @brief Return a DOT language representation of this vop node.
     *
     * This method allows visual working memory to recursively construct a DOT language
     * representation of the visual operation graph.
     */
    std::string get_dot_string();
};


typedef std::unordered_map<int, visual_operation_node*>          id_node_map;
#endif
