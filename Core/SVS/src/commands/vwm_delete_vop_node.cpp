/**
 * delete_vop_node_command
 *
 * Soar command to delete a visual operation node and its children from the
 * visual operation graph.
 *
 * Parameters:
 *     ^node-id <int> - the id of the visual operation node to delete
 *
 * See `visual_operation.h` for more information on the visual operations,
 * including which operations are available and what parameters they take.
*/

#ifdef ENABLE_OPENCV
// SVS includes
#include "svs.h"
#include "command.h"
#include "command_table.h"
#include "visual_operation.h"


/////////////////
// DECLARATION //
/////////////////
class delete_vop_node_command : public command
{
private:
    Symbol* root;
    soar_interface* si;
    svs_state* state;
    visual_working_memory* vwm;
public:
    delete_vop_node_command(svs_state* state, Symbol* root);
    ~delete_vop_node_command();
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};


/////////////////
// DEFINITIONS //
/////////////////
delete_vop_node_command::delete_vop_node_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    vwm = state->get_vwm();
}

delete_vop_node_command::~delete_vop_node_command() {};

std::string delete_vop_node_command::description() {
    return std::string("Deletes a visual operation node and its children from the visual operation graph.");
}

bool delete_vop_node_command::update_sub() {
    std::string status;
    wme_vector children;

    // Get the node id
    long node_id = 0;
    if (!si->get_const_attr(root, "node-id", node_id)) {
        set_status("no node-id specified");
        return false;
    }

    // Delete the node
    int result = vwm->remove_visual_operation(node_id);
    if (result == -1) {
        set_status("no such node");
        return false;
    }

    set_status("success");
    return true;
}

command* _make_delete_vop_node_command_(svs_state* state, Symbol* root)
{
    return new delete_vop_node_command(state, root);
}

command_table_entry* delete_vop_node_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "delete-vop-node";
    e->description = "Deletes a visual operation node and its children from the visual operation graph.";
    e->create = &_make_delete_vop_node_command_;
    return e;
}
#endif // ENABLE_OPENCV
