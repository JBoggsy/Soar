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

class recall_command : public command {
private:
    svs_state* state;
    Symbol* root;
    soar_interface* si;
    visual_working_memory* vwm;
    visual_long_term_memory<opencv_image, exact_visual_archetype>* v_mem;

public:
    recall_command(svs_state* state, Symbol* root);
    
    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};
#endif

//////////////////
// DEFINITIONS //
////////////////
#ifdef ENABLE_OPENCV

recall_command::recall_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) {
    si = state->get_svs()->get_soar_interface();
    vwm = state->get_vwm();
    v_mem = state->get_svs()->get_v_mem_opencv();
}

std::string recall_command::description() {
    return std::string("Recall a percept from visual ltm to visual working memory.");
}

bool recall_command::update_sub() {
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
    v_mem->recall(id, &percept_copy);
    vwm->add_image(&percept_copy, id);
    vwm->move_vwme(id, x, y);

    set_status("success");
    return true;
}
#endif


//////////////////////////////
// COMMAND TABLE FUNCTIONS //
////////////////////////////

command* _make_recall_command_(svs_state* state, Symbol* root) {
    return new recall_command(state, root);
}

command_table_entry* recall_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "recall_command";
    e->description = "Recall a percept from visual ltm to visual working memory.";
    e->create = &_make_recall_command_;
    e->parameters["id"] = "The ID of the archetype to imagine.";
    e->parameters["x"] = "The x coordinate to place the imagined percept.";
    e->parameters["y"] = "The y coordinate to place the imagined percept.";
    return e;
}