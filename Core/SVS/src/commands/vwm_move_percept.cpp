#include <string>
#include "command.h"
#include "command_table.h"
#include "svs.h"
#include "symbol.h"
#include "image.h"
#include "visual_long_term_memory.h"


///////////////////
// DECLARATIONS //
/////////////////
#ifdef ENABLE_OPENCV

class move_percept_command : public command {
private:
    svs_state*              state;
    Symbol*                 root;
    soar_interface*         si;
    visual_working_memory*  vwm;
    visual_long_term_memory<opencv_image, exact_visual_archetype>* v_mem;

public:
    move_percept_command(svs_state* state, Symbol* root);
    
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};

#endif

//////////////////
// DEFINITIONS //
////////////////
#ifdef ENABLE_OPENCV

move_percept_command::move_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    vwm = state->get_vwm();
    v_mem = state->get_svs()->get_v_mem_opencv();
}

std::string move_percept_command::description() {
    return std::string("Move an imagined percept to the specified set of coordinates.");
}

bool move_percept_command::update_sub() {
    std::string id;
    long int new_x;
    long int new_y;

    if (!si->get_const_attr(root, "id", id)) {
        set_status("No vwme ID specified");
        return false;
    }

    if (!si->get_const_attr(root, "x", new_x)) {
        set_status("No x coordinate specified");
        return false;
    }

    if (!si->get_const_attr(root, "y", new_y)) {
        set_status("No y coordinate specified");
        return false;
    }

    vwm->move_vwme(id, new_x, new_y);
    set_status("success");
    return true;
    
}
#endif

//////////////////////////////
// COMMAND TABLE FUNCTIONS //
////////////////////////////

command* _make_move_percept_command_(svs_state* state, Symbol* root) {
    return new move_percept_command(state, root);
}

command_table_entry* move_percept_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "move-percept";
    e->description = "Move a vwme to the specified set of coordinates.";
    e->create = &_make_move_percept_command_;
    e->parameters["id"] = "The unique string ID of the wme to move.";
    e->parameters["x"] = "The x coordinate of the destination.";
    e->parameters["y"] = "The y coordinate of the destination.";
    return e;
}