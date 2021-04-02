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
class set_file_command : public command
{
private:

    svs_state*          state;
    Symbol*             root;
    soar_interface*     si;
    visual_sensory_memory*   vsm;

public:
    set_file_command(svs_state* state, Symbol* root);
    ~set_file_command();

    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};


/////////////////
// DEFINITIONS //
/////////////////
set_file_command::set_file_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) 
{
    si = state->get_svs()->get_soar_interface();
    vsm = state->get_svs()->get_vsm();
}

set_file_command::~set_file_command() {}

std::string set_file_command::description() {
    return std::string("Sets the file path SVS will load percepts from.");
}

bool set_file_command::update_sub() {
    std::string filepath;
    if (!si->get_const_attr(root, "filepath", filepath)) {
        set_status("no filepath specified");
        return false;
    }

    vsm->setfile(filepath);
    set_status("success");
    return true;
}

command* _make_set_file_command_(svs_state* state, Symbol* root)
{
    return new set_file_command(state, root);
}

command_table_entry* set_file_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "set-file";
    e->description = "Sets the file path SVS will load percepts from.";
    e->parameters["filepath"] = "The file path SVS will load percepts from.";
    e->create = &_make_set_file_command_;
    return e;
}