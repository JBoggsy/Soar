#include <string>
#include <sstream>
#include <stdio.h>
#include <map>
#include <vector>
#include "command.h"
#include "scene.h"
#include "command_table.h"
#include "svs.h"
#include "symbol.h"


//////////////////
// DECLARATIONS //
//////////////////
class save_percept_command : public command
{
private:
    svs_state*          state;
    Symbol*             root;
    soar_interface*     si;
    visual_sensory_memory*   vsm;

public:
    save_percept_command(svs_state* state, Symbol* root);
    ~save_percept_command();

    int command_type() { return SVS_READ_COMMAND; }
    std::string description();
    bool update_sub();
};


/////////////////
// DEFINITIONS //
/////////////////
save_percept_command::save_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) 
{
    si = state->get_svs()->get_soar_interface();
    vsm = state->get_svs()->get_vsm();
}

save_percept_command::~save_percept_command() {}

std::string save_percept_command::description() {
    return std::string("Save the current visual percept to the given file.");
}

bool save_percept_command::update_sub() {
    std::string filepath;
    if (!si->get_const_attr(root, "filepath", filepath)) {
        set_status("No filepath specified");
        return false;
    }

    vsm->save(filepath);
    set_status("success");
    return true;
}

command* _make_save_percept_command_(svs_state* state, Symbol* root)
{
    return new save_percept_command(state, root);
}

command_table_entry* save_percept_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "save_percept";
    e->description = "Save the current visual percept to the given file.";
    e->parameters["filepath"] = "The file path SVS will save the percept to.";
    e->create = &_make_save_percept_command_;
    return e;
}