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
class add_vop_node_command : public command
{
private:
    Symbol* root;
    soar_interface* si;
    svs_state* state;
    visual_sensory_memory* vsm;
public:
    add_vop_node_command(svs_state* state, Symbol* root);
    ~add_vop_node_command();
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};


/////////////////
// DEFINITIONS //
/////////////////

add_vop_node_command::add_vop_node_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    vsm = state->get_svs()->get_vsm();
}

add_vop_node_command::~add_vop_node_command() {};

std::string add_vop_node_command::description() {
    return std::string("Adds a new visual operation node to the visual operation graph.");
}

bool add_vop_node_command::update_sub() {
    std::string op_type;
    std::string status;
    wme_vector children;

    if (!si->get_const_attr(root, "op-type", op_type)) {
        set_status("no op-type specified");
        return false;
    }

    if (si->get_const_attr(root, "status", status)) {
        if (status.compare("success") == 0) {
            return false;
        }
    }

    visual_ops::vop_params_metadata node_params_metadata = visual_ops::vops_param_table[op_type];

    // Collect all of the parameter arguments into the nodes' data_dict and its parent node dict
    data_dict* node_data_dict_pointer = new data_dict;
    #define node_data_dict (*node_data_dict_pointer)
    std::unordered_map<std::string, int> node_parents;
    int num_params = node_params_metadata.num_params;
    for (int param_i=0; param_i < num_params; param_i++) {
        
        std::string param_name = node_params_metadata.param_names.at(param_i);
        visual_ops::ArgOptionality param_opt = node_params_metadata.param_optionalities.at(param_i);
        visual_ops::ArgDirection param_dir = node_params_metadata.param_direction.at(param_i);
        visual_ops::ArgType param_type = node_params_metadata.param_types.at(param_i);

        bool param_present = false;
        switch (param_type) {
            case visual_ops::INT_ARG:
                node_data_dict[param_name] = new long;
                param_present = si->get_const_attr(root, param_name, *((long*)node_data_dict[param_name]));
                break;
            case visual_ops::DOUBLE_ARG:
                node_data_dict[param_name] = new double;
                param_present = si->get_const_attr(root, param_name, *((double*)node_data_dict[param_name]));
                break;
            case visual_ops::STRING_ARG:
                node_data_dict[param_name] = new std::string;
                param_present = si->get_const_attr(root, param_name, *((std::string*)node_data_dict[param_name]));
                break;
            case visual_ops::NODE_ID_ARG:
                node_data_dict[param_name] = new long;
                param_present = si->get_const_attr(root, param_name, *((long*)node_data_dict[param_name]));
                node_parents[param_name] = (int)*((long*)node_data_dict[param_name]);
                break;
            case visual_ops::VSM_ARG:
                node_data_dict[param_name] = vsm;
                param_present = true;
                break;
            case visual_ops::VWM_ARG:
                node_data_dict[param_name] = state->get_vwm();
                param_present = true;
                break;
            case visual_ops::VLTM_ARG:
                node_data_dict[param_name] = state->get_svs()->get_v_mem_opencv();
                param_present = true;
                break;
            default:
                node_data_dict[param_name] = NULL;
                param_present = false;
                break;
        }

        // Decide what to do with an argument whose value is missing
        if (!param_present) {
            if (param_opt == visual_ops::REQUIRED_ARG && param_dir == visual_ops::INPUT_ARG) {  // error out if param was required
                char status_buffer[64];
                sprintf(status_buffer, "missing req'd param %s\n", param_name.c_str());
                set_status(std::string(status_buffer));
                return false;
            } else if (param_type == visual_ops::NODE_ID_ARG) {
                node_data_dict[param_name] = new long(-1);
                node_parents[param_name] = -1;
            } else if (param_dir != visual_ops::INPUT_ARG) {  // outputs must be allocated for later use
                switch (param_type) {
                    case visual_ops::INT_ARG:
                        node_data_dict[param_name] = new long;
                        break;
                    case visual_ops::DOUBLE_ARG:
                        node_data_dict[param_name] = new double;
                        break;
                    case visual_ops::STRING_ARG:
                        node_data_dict[param_name] = new std::string;
                        break;
                }
            } else {
                node_data_dict[param_name] = NULL;
            }
        }
    }

    int node_insertion_result = vsm->get_vop_graph()->insert(op_type, node_data_dict_pointer, node_parents);
    if (node_insertion_result == -1) {
        set_status("no valid parent specified");
        return false;
    }

    set_status("success");
    return true;
}

command* _make_add_vop_node_command_(svs_state* state, Symbol* root)
{
    return new add_vop_node_command(state, root);
}

command_table_entry* add_vop_node_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "add-vop-node";
    e->description = "Adds a new visual operation node to the visual operation graph.";
    e->create = &_make_add_vop_node_command_;
    return e;
}
#endif