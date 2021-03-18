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

class imagine_translate_percept_command : public command {
private:
    svs_state* state;
    Symbol* root;
    soar_interface* si;
    imagination_opencv* imagination;
    visual_long_term_memory<opencv_image, exact_visual_archetype>* v_mem;

public:
    imagine_translate_percept_command(svs_state* state, Symbol* root);
    
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};

#endif

//////////////////
// DEFINITIONS //
////////////////
#ifdef ENABLE_OPENCV

imagine_translate_percept_command::imagine_translate_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    // imagination = state->get_imagination();
    v_mem = state->get_svs()->get_v_mem_opencv();
}

std::string imagine_translate_percept_command::description() {
    return std::string("Translate an imagined percept the specified distances on the x and y axes.");
}

bool imagine_translate_percept_command::update_sub() {
    long int id;
    long int d_x;
    long int d_y;

    if (!si->get_const_attr(root, "id", id)) {
        set_status("No imagined percept ID specified");
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

    imagination->translate_percept(id, d_x, d_y);
    set_status("success");
    return true; 
}

#endif


//////////////////////////////
// COMMAND TABLE FUNCTIONS //
////////////////////////////

command* _make_imagine_translate_percept_command_(svs_state* state, Symbol* root) {
    return new imagine_translate_percept_command(state, root);
}

command_table_entry* imagine_translate_percept_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "imagine_translate_percept";
    e->description = "Translate an imagined percept along the x and y axes.";
    e->create = &_make_imagine_translate_percept_command_;
    e->parameters["id"] = "The unique integer ID of the imagined percept to translate.";
    e->parameters["x"] = "The distance to translate the percept along the x axis.";
    e->parameters["y"] = "The distance to translate the percept along the y axis.";
    return e;
}