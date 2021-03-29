// #include "svs.h"
// #include "symbol.h"
// #include "vision_interface.h"
// #include "visual_long_term_memory.h"
// #include "command.h"
// #include "command_table.h"


// //////////////////
// // DECLARATION //
// ////////////////

// class match_percept_command : public command {
// private:
//     svs_state*          state;
//     Symbol*             root;
//     soar_interface*     si;
//     vision_interface*   vi;

// public:
//     match_percept_command(svs_state* state, Symbol* root);
//     ~match_percept_command();

//     int command_type() { return SVS_WRITE_COMMAND; }
//     std::string description();
//     bool update_sub();

// };

// //////////////////
// // DEFINITIONS //
// ////////////////
// match_percept_command::match_percept_command(svs_state* state, Symbol* root)
//     : command(state, root), state(state), root(root) 
// {
//     si = state->get_svs()->get_soar_interface();
//     vi = state->get_svs()->get_vision_interface();
// }

// match_percept_command::~match_percept_command() {}

// std::string match_percept_command::description() {
//     return std::string("Retrieve from visual memory the visual archetype most similar to the current perecpt.");
// }

// bool match_percept_command::update_sub() {
//     vmem_match percept_match;
//     vi->match(&percept_match);

//     wme* match_id_wme = si->make_wme(root, "id", percept_match.entity_id);
//     wme* match_conf_wme = si->make_wme(root, "confidence", percept_match.confidence);
//     set_status("success");
//     return true;
// }

// command* _make_match_percept_command_(svs_state* state, Symbol* root)
// {
//     return new match_percept_command(state, root);
// }

// command_table_entry* match_percept_command_entry()
// {
//     command_table_entry* e = new command_table_entry();
//     e->name = "match_percept";
//     e->description = "Retrieve from visual memory the visual archetype most similar to the current perecpt.";
//     e->create = &_make_match_percept_command_;
//     return e;
// }