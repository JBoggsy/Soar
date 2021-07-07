#ifdef ENABLE_OPENCV

#include "svs.h"
#include "symbol.h"
#include "visual_long_term_memory.h"
#include "command.h"
#include "command_table.h"


//////////////////
// DECLARATION //
////////////////

class remember_percept_command : public command {
private:
    svs_state*              state;
    Symbol*                 root;
    soar_interface*         si;
    visual_working_memory*  vwm;
    visual_long_term_memory<opencv_image, exact_visual_archetype>* v_mem;

public:
    remember_percept_command(svs_state* state, Symbol* root);
    ~remember_percept_command();

    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();

};

//////////////////
// DEFINITIONS //
////////////////
remember_percept_command::remember_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) 
{
    si = state->get_svs()->get_soar_interface();
    vwm = state->get_vwm();
    v_mem = state->get_svs()->get_v_mem_opencv();
}

remember_percept_command::~remember_percept_command() {}

std::string remember_percept_command::description() {
    return std::string("Remember the visual percept represented by VWM as an example of the given archetype.");
}

bool remember_percept_command::update_sub() {
    std::string id;
    if (!si->get_const_attr(root, "id", id)) {
        set_status("No archetype ID specified");
        return false;
    }

    opencv_image* vwm_percept = vwm->get_percept();
    v_mem->store_percept(vwm_percept, id);

    set_status("success");
    return true;
}

command* _make_remember_percept_command_(svs_state* state, Symbol* root)
{
    return new remember_percept_command(state, root);
}

command_table_entry* remember_percept_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "remember-percept";
    e->description = "Remember the current visual percept represented by VWM as an example of the given archetype.";
    e->parameters["id"] = "The ID of the archetype the percept exemplifies.";
    e->create = &_make_remember_percept_command_;
    return e;
}
#endif