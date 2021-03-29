#include "svs.h"
#include "symbol.h"
#include "vision_interface.h"
#include "command.h"
#include "command_table.h"


//////////////////
// DECLARATION //
////////////////

class recall_percept_command : public command {
private:
    svs_state*          state;
    Symbol*             root;
    soar_interface*     si;
    vision_interface*   vi;

public:
    recall_percept_command(svs_state* state, Symbol* root);
    ~recall_percept_command();

    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();

};

//////////////////
// DEFINITIONS //
////////////////
recall_percept_command::recall_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) 
{
    si = state->get_svs()->get_soar_interface();
    vi = state->get_svs()->get_vision_interface();
}

recall_percept_command::~recall_percept_command() {}

std::string recall_percept_command::description() {
    return std::string("Recall and put into visual input the visual percept with the given archetype ID.");
}

bool recall_percept_command::update_sub() {
    std::string id;
    if (!si->get_const_attr(root, "id", id)) {
        set_status("No archetype ID specified");
        return false;
    }

    vi->recall(id);
    set_status("success");
    return true;
}

command* _make_recall_percept_command_(svs_state* state, Symbol* root)
{
    return new recall_percept_command(state, root);
}

command_table_entry* recall_percept_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "recall_percept";
    e->description = "Recall and put into visual input the visual percept with the given archetype ID.";
    e->parameters["id"] = "The ID of the archetype to recall.";
    e->create = &_make_recall_percept_command_;
    return e;
}