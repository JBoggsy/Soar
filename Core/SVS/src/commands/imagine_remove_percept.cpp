#include <string>
#include "command.h"
#include "command_table.h"
#include "svs.h"
#include "symbol.h"
#include "image.h"
#include "visual_memory.h"


///////////////////
// DECLARATIONS //
/////////////////
#ifdef ENABLE_OPENCV

class imagine_remove_percept_command : public command {
private:
    svs_state* state;
    Symbol* root;
    soar_interface* si;
    imagination_opencv* imagination;
    visual_memory<opencv_image, exact_visual_archetype>* v_mem;

public:
    imagine_remove_percept_command(svs_state* state, Symbol* root);
    
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};

#endif

//////////////////
// DEFINITIONS //
////////////////
#ifdef ENABLE_OPENCV

imagine_remove_percept_command::imagine_remove_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    imagination = state->get_imagination();
    v_mem = state->get_svs()->get_v_mem_opencv();
}

std::string imagine_remove_percept_command::description() {
    return std::string("Remove an imagined percept.");
}

bool imagine_remove_percept_command::update_sub() {
    long int id;

    if (!si->get_const_attr(root, "id", id)) {
        set_status("No imagined percept ID specified");
        return false;
    }

    imagination->remove_percept(id);
    set_status("success");
    return true;
}

#endif


//////////////////////////////
// COMMAND TABLE FUNCTIONS //
////////////////////////////

command* _make_imagine_remove_percept_command_(svs_state* state, Symbol* root) {
    return new imagine_remove_percept_command(state, root);
}

command_table_entry* imagine_remove_percept_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "imagine_remove_percept";
    e->description = "Remove an imagined percept.";
    e->create = &_make_imagine_remove_percept_command_;
    e->parameters["id"] = "The unique integer ID of the imagined percept to remove.";
    return e;
}