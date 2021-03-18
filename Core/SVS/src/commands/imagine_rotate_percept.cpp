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

class imagine_rotate_percept_command : public command {
private:
    svs_state* state;
    Symbol* root;
    soar_interface* si;
    imagination_opencv* imagination;
    visual_long_term_memory<opencv_image, exact_visual_archetype>* v_mem;

public:
    imagine_rotate_percept_command(svs_state* state, Symbol* root);
    
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};

#endif

//////////////////
// DEFINITIONS //
////////////////
#ifdef ENABLE_OPENCV

imagine_rotate_percept_command::imagine_rotate_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    // imagination = state->get_imagination();
    v_mem = state->get_svs()->get_v_mem_opencv();
}

std::string imagine_rotate_percept_command::description() {
    return std::string("Flip/mirror an imagined percept along the x and/or y axis.");
}

bool imagine_rotate_percept_command::update_sub() {
    long int id;
    double amount;

    if (!si->get_const_attr(root, "id", id)) {
        set_status("No imagined percept ID specified");
        return false;
    }

    if (!si->get_const_attr(root, "amount", amount)) {
        set_status("No rotation amount specified");
        return false;
    }

    imagination->rotate_percept_deg(id, amount);
    set_status("success");
    return true;
}

#endif


//////////////////////////////
// COMMAND TABLE FUNCTIONS //
////////////////////////////

command* _make_imagine_rotate_percept_command_(svs_state* state, Symbol* root) {
    return new imagine_rotate_percept_command(state, root);
}

command_table_entry* imagine_rotate_percept_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "imagine_rotate_percept";
    e->description = "Rotate an imagined percept by the specified amount.";
    e->create = &_make_imagine_rotate_percept_command_;
    e->parameters["id"] = "The unique integer ID of the imagined percept to rotate.";
    e->parameters["amount"] = "The number of degrees to rotate the percept, with positive being clockwise.";
    return e;
}