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

class translate_percept_command : public command {
private:
    svs_state* state;
    Symbol* root;
    soar_interface* si;
    visual_working_memory* vwm;

public:
    translate_percept_command(svs_state* state, Symbol* root);
    
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};

#endif

//////////////////
// DEFINITIONS //
////////////////
#ifdef ENABLE_OPENCV

translate_percept_command::translate_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    vwm = state->get_vwm();
}

std::string translate_percept_command::description() {
    return std::string("Translate a vwme the specified distances on the x and y axes.");
}

bool translate_percept_command::update_sub() {
    std::string id;
    long int d_x;
    long int d_y;

    if (!si->get_const_attr(root, "id", id)) {
        set_status("No vwme ID specified");
        return false;
    }

    if (!si->get_const_attr(root, "x", d_x)) {
        set_status("No x coordinate specified");
        return false;
    }

    if (!si->get_const_attr(root, "y", d_y)) {
        set_status("No y coordinate specified");
        return false;
    }

    vwm->translate_vwme(id, d_x, d_y);
    set_status("success");
    return true; 
}

#endif


//////////////////////////////
// COMMAND TABLE FUNCTIONS //
////////////////////////////

command* _make_translate_percept_command_(svs_state* state, Symbol* root) {
    return new translate_percept_command(state, root);
}

command_table_entry* translate_percept_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "translate-percept";
    e->description = "Translate an imagined percept along the x and y axes.";
    e->create = &_make_translate_percept_command_;
    e->parameters["id"] = "The unique string ID of the imagined percept to translate.";
    e->parameters["x"] = "The distance to translate the percept along the x axis.";
    e->parameters["y"] = "The distance to translate the percept along the y axis.";
    return e;
}