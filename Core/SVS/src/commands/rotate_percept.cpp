#include "svs.h"
#include "symbol.h"
#include "vision_interface.h"
#include "visual_long_term_memory.h"
#include "command.h"
#include "command_table.h"


//////////////////
// DECLARATION //
////////////////

class rotate_percept_command : public command {
private:
    svs_state*          state;
    Symbol*             root;
    soar_interface*     si;
    vision_interface*   vi;

public:
    rotate_percept_command(svs_state* state, Symbol* root);
    ~rotate_percept_command();

    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();

};

//////////////////
// DEFINITIONS //
////////////////
rotate_percept_command::rotate_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) 
{
    si = state->get_svs()->get_soar_interface();
    vi = state->get_svs()->get_vision_interface();
}

rotate_percept_command::~rotate_percept_command() {}

std::string rotate_percept_command::description() {
    return std::string("Rotate the current visual percept by the specified amount.");
}

bool rotate_percept_command::update_sub() {
    long int amount;
    if (!si->get_const_attr(root, "amount", amount)) {
        set_status("No rotation amount specified");
        return false;
    }
    vi->rotate(amount);
    set_status("success");
    return true;
}

command* _make_rotate_percept_command_(svs_state* state, Symbol* root)
{
    return new rotate_percept_command(state, root);
}

command_table_entry* rotate_percept_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "rotate_percept";
    e->description = "Rotate the current visual percept by the specified amount.";
    e->parameters["amount"] = "The amount in degrees to rotate the image. Must be 90, 180, or 270.";
    e->create = &_make_rotate_percept_command_;
    return e;
}