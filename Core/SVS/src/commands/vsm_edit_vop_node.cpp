#ifdef ENABLE_OPENCV

// C++ standard library

// Soar kernal

// SVS includes
#include "svs.h"
#include "command.h"
#include "command_table.h"
#include "visual_operation.h"


/////////////////
// DECLARATION //
/////////////////
class edit_vop_node_command : public command
{
private:
    Symbol* root;
    soar_interface* si;
    svs_state* state;
    visual_sensory_memory* vsm;
public:
    edit_vop_node_command(svs_state* state, Symbol* root);
    ~edit_vop_node_command();
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};


/////////////////
// DEFINITIONS //
/////////////////

edit_vop_node_command::edit_vop_node_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    vsm = state->get_svs()->get_vsm();
}

edit_vop_node_command::~edit_vop_node_command() {};

std::string edit_vop_node_command::description() {
    return std::string("Edits one or more argument values of a visual operation node.");
}

bool edit_vop_node_command::update_sub() {
    long node_id;
    if (!si->get_const_attr(root, "node-id", node_id)) {
        set_status("no node-id specified");
        return false;
    }

    visual_operation_node* vop_node = vsm->get_vop_graph()->get_node(node_id);
    if (vop_node == NULL) { 
        set_status("cannot find node-id specified");
        return false;
    }

    std::string vop_type = vop_node->get_op_type();
    visual_ops::vop_params_metadata vop_params_metadata = visual_ops::vops_param_table[vop_type];
    int num_params = vop_params_metadata.num_params;
    void* new_value;
    for (int param_i=0; param_i < num_params; param_i++) {
        std::string param_name = vop_params_metadata.param_names.at(param_i);
        visual_ops::ArgType param_type = vop_params_metadata.param_types.at(param_i);

        new_value = NULL;
        switch (param_type) {
            case visual_ops::INT_ARG:
                new_value = new long;
                if (si->get_const_attr(root, param_name, *((long*)new_value))) {
                    vop_node->edit_parameter(param_name, *((long*)new_value));
                }
                break;
            case visual_ops::NODE_ID_ARG:
                new_value = new long;
                if (si->get_const_attr(root, param_name, *((long*)new_value))) {
                    vop_node->edit_parameter(param_name, *((long*)new_value));
                }
                break;
            case visual_ops::DOUBLE_ARG:
                new_value = new double;
                if (si->get_const_attr(root, param_name, *((double*)new_value))) {
                    vop_node->edit_parameter(param_name, *((double*)new_value));
                }
                break;
            case visual_ops::STRING_ARG:
                new_value = new std::string;
                if (si->get_const_attr(root, param_name, *((std::string*)new_value))) {
                    vop_node->edit_parameter(param_name, *((std::string*)new_value));
                }
                break;
        }
    }
    set_status("success");
    return true;
}

command* _make_edit_vop_node_command_(svs_state* state, Symbol* root)
{
    return new edit_vop_node_command(state, root);
}

command_table_entry* edit_vop_node_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "edit-vop-node";
    e->description = "Edits one or more argument values of a visual operation node.";
    e->create = &_make_edit_vop_node_command_;
    return e;
}
#endif