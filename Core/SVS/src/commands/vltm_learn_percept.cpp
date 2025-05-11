/**
 * @file vltm_learn_percept.cpp
 * @brief Implementation of the VLTM learn percept command
 *
 * learn_percept_command
 *
 * Soar Command to learn the percept output by a specified visual operation node
 * and store it in the VLTM VCD associated with the specified name. If the no
 * VCD with the specified name exists, a new one is created, otherwise the
 * existing VCD is updated.
 *
 * Parameters:
 *     ^name <string> - the name to be associated with the percept
 *     ^node-id <int> - the ID of the visual operation node whose percept is to
 *     be learned
 */

#ifdef ENABLE_OPENCV
// SVS includes
#include "svs.h"
#include "command.h"
#include "command_table.h"
#include "latent_representation.h"
#include "token_sequence.h"
#include "visual_long_term_memory.h"

/////////////////
// DECLARATION //
/////////////////
class learn_percept_command : public command
{
private:
    Symbol* root;
    soar_interface* si;
    svs_state* state;
    visual_working_memory* vwm;
    VLTM_TYPE* vltm;
public:
    learn_percept_command(svs_state* state, Symbol* root);
    ~learn_percept_command();
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};


/////////////////
// DEFINITIONS //
/////////////////
learn_percept_command::learn_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    vwm = state->get_vwm();
    vltm = state->get_svs()->get_vltm();
}

learn_percept_command::~learn_percept_command() {};

std::string learn_percept_command::description() {
    return std::string("Stores the `source` image of a VOp node in VLTM as a VCD with the specified name.");
}

bool learn_percept_command::update_sub() {
    std::string name;
    long node_id;
    std::string status;
    wme_vector children;

    if (!si->get_const_attr(root, "name", name)) {
        set_status("no name specified");
        return false;
    }

    if (!si->get_const_attr(root, "node-id", node_id)) {
        set_status("no node-id specified");
        return false;
    }

    if (si->get_const_attr(root, "status", status)) {
        if (status.compare("success") == 0) {
            return false;
        }
    }

    visual_operation_node* node = vwm->get_node(node_id);
    if (node == NULL) {
        set_status("no node with that ID");
        return false;
    }

    opencv_image* percept = node->get_node_image();
    if (percept == NULL) {
        set_status("node has no image");
        return false;
    }

    #ifdef ENABLE_TORCH
    // TODO: I should make this more flexible inthe final code so that it can be
    // chosen at runtime.
    // latent_representation* latent = new latent_representation();
    token_sequence* representation = new token_sequence(49, 256);

    vltm->encode_image(percept, representation);
    vltm->store_percept(representation, name);
    #else
    vltm->store_percept(percept, name);
    #endif

    set_status("success");

    return true;
}

command* _make_learn_percept_command_(svs_state* state, Symbol* root)
{
    return new learn_percept_command(state, root);
}

command_table_entry* learn_percept_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "learn-percept";
    e->description = "Stores the `source` image of a VOp node in VLTM as a VCD with the specified name.";
    e->create = &_make_learn_percept_command_;
    return e;
}

#endif
