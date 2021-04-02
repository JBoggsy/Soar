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

class flip_percept_command : public command {
private:
    svs_state* state;
    Symbol* root;
    soar_interface* si;
    visual_working_memory* vwm;
    visual_long_term_memory<opencv_image, exact_visual_archetype>* v_mem;

public:
    flip_percept_command(svs_state* state, Symbol* root);
    
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};

#endif

//////////////////
// DEFINITIONS //
////////////////
#ifdef ENABLE_OPENCV

flip_percept_command::flip_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    vwm = state->get_vwm();
    v_mem = state->get_svs()->get_v_mem_opencv();
}

std::string flip_percept_command::description() {
    return std::string("Flip/mirror an imagined percept along the x and/or y axis.");
}

bool flip_percept_command::update_sub() {
    std::string id;
    std::string axes;
    bool x;
    bool y;

    if (!si->get_const_attr(root, "id", id)) {
        set_status("No visual wme ID specified");
        return false;
    }

    if (!si->get_const_attr(root, "axes", axes)) {
        set_status("No axes specified");
        return false;
    }

    x = (axes.find("x") != std::string::npos);  // `x` is true if "x" is in `axes`
    y = (axes.find("y") != std::string::npos);  // same for `y`

    if (x) { vwm->flip_vwme_horiz(id); }
    if (y) { vwm->flip_vwme_vert(id); }

    set_status("success");
    return true;
}
#endif


//////////////////////////////
// COMMAND TABLE FUNCTIONS //
////////////////////////////

command* _make_flip_percept_command_(svs_state* state, Symbol* root) {
    return new flip_percept_command(state, root);
}

command_table_entry* flip_percept_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "flip-percept";
    e->description = "Flip/mirror a vwme percept along the x and/or y axis.";
    e->create = &_make_flip_percept_command_;
    e->parameters["id"] = "The unique string ID of the imagined percept to flip.";
    e->parameters["axes"] = "The axes to flip on. Should be one of \"x\", \"y\", \"xy\", or \"yx\".";
    return e;
}