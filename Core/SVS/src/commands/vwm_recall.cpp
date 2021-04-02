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
    std::string arch_id;
    std::string vwme_id;

    if (!si->get_const_attr(root, "arch_id", arch_id)) {
        set_status("No archetype ID specified");
        return false;
    }
    if (!si->get_const_attr(root, "vwme_id", vwme_id)) {
        set_status("No VWME ID specified");
        return false;
    }

    opencv_image percept_copy = opencv_image();
    v_mem->recall(arch_id, &percept_copy);
    vwm->add_image(&percept_copy, vwme_id);

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
    e->name = "recall-percept";
    e->description = "Recall a percept from visual ltm to visual working memory.";
    e->create = &_make_recall_command_;
    e->parameters["arch-id"] = "The ID of the archetype to imagine.";
    e->parameters["vwme-id"] = "The unique string ID of the resulting vwme.";
    return e;
}