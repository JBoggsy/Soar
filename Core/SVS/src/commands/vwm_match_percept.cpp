#ifdef ENABLE_OPENCV

#include "svs.h"
#include "symbol.h"
#include "vision_interface.h"
#include "visual_long_term_memory.h"
#include "visual_matching.h"
#include "command.h"
#include "command_table.h"


//////////////////
// DECLARATION //
////////////////

class match_percept_command : public command {
private:
    svs_state*              state;
    Symbol*                 root;
    soar_interface*         si;
    visual_working_memory*  vwm;
    visual_long_term_memory<opencv_image, exact_visual_archetype>* vltm;

public:
    match_percept_command(svs_state* state, Symbol* root);
    ~match_percept_command();

    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();

};

//////////////////
// DEFINITIONS //
////////////////
match_percept_command::match_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) 
{
    si = state->get_svs()->get_soar_interface();
    vwm = state->get_vwm();
    vltm = state->get_svs()->get_v_mem_opencv();
}

match_percept_command::~match_percept_command() {}

std::string match_percept_command::description() {
    return std::string("Retrieve from visual memory the visual archetype most similar to the perceptual representation of VWM.");
}

bool match_percept_command::update_sub() {
    vmem_match percept_match;
    opencv_image* vwm_percept = vwm->get_percept();
    vltm->match(vwm_percept, visual_matching::opencv::simple_template_compare, &percept_match);

    wme* match_id_wme = si->make_wme(root, "id", percept_match.entity_id);
    wme* match_conf_wme = si->make_wme(root, "confidence", percept_match.confidence);
    set_status("success");
    return true;
}

command* _make_match_percept_command_(svs_state* state, Symbol* root)
{
    return new match_percept_command(state, root);
}

command_table_entry* match_percept_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "match-percept";
    e->description = "Retrieve from visual memory the visual archetype most similar to the perceptual representation of VWM.";
    e->create = &_make_match_percept_command_;
    return e;
}
#endif