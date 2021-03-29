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

class imagine_add_percept_command : public command {
private:
    svs_state* state;
    Symbol* root;
    soar_interface* si;
    // imagination_opencv* imagination;
    visual_long_term_memory<opencv_image, exact_visual_archetype>* v_mem;

public:
    imagine_add_percept_command(svs_state* state, Symbol* root);
    
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};
#endif

//////////////////
// DEFINITIONS //
////////////////
#ifdef ENABLE_OPENCV

imagine_add_percept_command::imagine_add_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    // imagination = state->get_imagination();
    v_mem = state->get_svs()->get_v_mem_opencv();
}

std::string imagine_add_percept_command::description() {
    return std::string("Add a new imagined percept to the imagined scene.");
}

bool imagine_add_percept_command::update_sub() {
    std::string id;
    long int x;
    long int y;

    if (!si->get_const_attr(root, "id", id)) {
        set_status("No archetype ID specified");
        return false;
    }

    if (!si->get_const_attr(root, "x", x)) {
        x = 0;
    }
    if (!si->get_const_attr(root, "y", y)) {
        y = 0;
    }

    opencv_image percept_copy = opencv_image();
    state->get_svs()->get_v_mem_opencv()->recall(id, &percept_copy);
    // imagination->add_percept(percept_copy, x, y);

    set_status("success");
    return true;
}
#endif


//////////////////////////////
// COMMAND TABLE FUNCTIONS //
////////////////////////////

command* _make_imagine_add_percept_command_(svs_state* state, Symbol* root) {
    return new imagine_add_percept_command(state, root);
}

command_table_entry* imagine_add_percept_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "imagine_add_percept";
    e->description = "Add a new imagined percept to the imagined scene.";
    e->create = &_make_imagine_add_percept_command_;
    e->parameters["id"] = "The ID of the archetype to imagine.";
    e->parameters["x"] = "The x coordinate to place the imagined percept.";
    e->parameters["y"] = "The y coordinate to place the imagined percept.";
    return e;
}