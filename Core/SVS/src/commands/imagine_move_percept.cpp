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

class imagine_move_percept_command : public command {
private:
    svs_state* state;
    Symbol* root;
    soar_interface* si;
    imagination_opencv* imagination;
    visual_long_term_memory<opencv_image, exact_visual_archetype>* v_mem;

public:
    imagine_move_percept_command(svs_state* state, Symbol* root);
    
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};

#endif

//////////////////
// DEFINITIONS //
////////////////
#ifdef ENABLE_OPENCV

imagine_move_percept_command::imagine_move_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    // imagination = state->get_imagination();
    v_mem = state->get_svs()->get_v_mem_opencv();
}

std::string imagine_move_percept_command::description() {
    return std::string("Move an imagined percept to the specified set of coordinates.");
}

bool imagine_move_percept_command::update_sub() {
    long int id;
    long int new_x;
    long int new_y;

    if (!si->get_const_attr(root, "id", id)) {
        set_status("No imagined percept ID specified");
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

    imagination->move_percept(id, new_x, new_y);
    set_status("success");
    return true;
    
}
#endif


//////////////////////////////
// COMMAND TABLE FUNCTIONS //
////////////////////////////

command* _make_imagine_move_percept_command_(svs_state* state, Symbol* root) {
    return new imagine_move_percept_command(state, root);
}


command_table_entry* imagine_move_percept_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "imagine_move_percept";
    e->description = "Move an imagined percept to the specified set of coordinates.";
    e->create = &_make_imagine_move_percept_command_;
    e->parameters["id"] = "The unique integer ID of the imagined percept to move.";
    e->parameters["x"] = "The x coordinate of the destination.";
    e->parameters["y"] = "The y coordinate of the destination.";
    return e;
}