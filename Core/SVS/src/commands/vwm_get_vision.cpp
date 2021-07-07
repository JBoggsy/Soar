#ifdef ENABLE_OPENCV

#include <string>
#include "command.h"
#include "command_table.h"
#include "svs.h"
#include "symbol.h"
#include "image.h"

///////////////////
// DECLARATIONS //
/////////////////

class get_vision_command : public command {
private:
    svs_state* state;
    Symbol* root;
    soar_interface* si;
    visual_working_memory* vwm;
    visual_sensory_memory* vsm;

public:
    get_vision_command(svs_state* state, Symbol* root);
    
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};
//////////////////
// DEFINITIONS //
////////////////

get_vision_command::get_vision_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    vwm = state->get_vwm();
    vsm = state->get_svs()->get_vsm();
}

std::string get_vision_command::description() {
    return std::string("Take the percept from VSM and add it as a VWME");
}

bool get_vision_command::update_sub() {
    std::string id;

    if (!si->get_const_attr(root, "id", id)) {
        set_status("No vwme ID specified");
        return false;
    }

    opencv_image* vision = vsm->give_vision();
    vwm->add_image(vision, id);
    set_status("success");
    return true;
}

//////////////////////////////
// COMMAND TABLE FUNCTIONS //
////////////////////////////

command* _make_get_vision_command_(svs_state* state, Symbol* root) {
    return new get_vision_command(state, root);
}

command_table_entry* get_vision_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "get-vision";
    e->description = "Take the percept from VSM and add it as a VWME with the given ID.";
    e->create = &_make_get_vision_command_;
    e->parameters["id"] = "The unique string ID of the added VWME.";
    return e;
}
#endif