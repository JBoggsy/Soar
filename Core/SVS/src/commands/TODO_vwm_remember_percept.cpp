// #include "svs.h"
// #include "symbol.h"
// #include "vision_interface.h"
// #include "command.h"
// #include "command_table.h"


// //////////////////
// // DECLARATION //
// ////////////////

// class remember_percept_command : public command {
// private:
//     svs_state*          state;
//     Symbol*             root;
//     soar_interface*     si;
//     vision_interface*   vi;

// public:
//     remember_percept_command(svs_state* state, Symbol* root);
//     ~remember_percept_command();

//     int command_type() { return SVS_WRITE_COMMAND; }
//     std::string description();
//     bool update_sub();

// };

// //////////////////
// // DEFINITIONS //
// ////////////////
// remember_percept_command::remember_percept_command(svs_state* state, Symbol* root)
//     : command(state, root), state(state), root(root) 
// {
//     si = state->get_svs()->get_soar_interface();
//     vi = state->get_svs()->get_vision_interface();
// }

// remember_percept_command::~remember_percept_command() {}

// std::string remember_percept_command::description() {
//     return std::string("Remember the current visual percept as an example of the given archetype.");
// }

// bool remember_percept_command::update_sub() {
//     std::string id;
//     if (!si->get_const_attr(root, "id", id)) {
//         set_status("No archetype ID specified");
//         return false;
//     }

//     vi->remember(id);
//     set_status("success");
//     return true;
// }

// command* _make_remember_percept_command_(svs_state* state, Symbol* root)
// {
//     return new remember_percept_command(state, root);
// }

// command_table_entry* remember_percept_command_entry()
// {
//     command_table_entry* e = new command_table_entry();
//     e->name = "remember_percept";
//     e->description = "Remember the current visual percept as an example of the given archetype.";
//     e->parameters["id"] = "The ID of the archetype the percept exemplifies.";
//     e->create = &_make_remember_percept_command_;
//     return e;
// }