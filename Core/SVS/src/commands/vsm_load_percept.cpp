#ifdef ENABLE_OPENCV

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
class load_percept_command : public command
{
private:

    svs_state*          state;
    Symbol*             root;
    soar_interface*     si;
    visual_sensory_memory* vsm;

public:
    load_percept_command(svs_state* state, Symbol* root);
    ~load_percept_command();

    int command_type() { return SVS_WRITE_COMMAND; }
    std::string description();
    bool update_sub();
};


/////////////////
// DEFINITIONS //
/////////////////
load_percept_command::load_percept_command(svs_state* state, Symbol* root)
    : command(state, root), state(state), root(root) 
{
    si = state->get_svs()->get_soar_interface();
    vsm = state->get_svs()->get_vsm();
}

load_percept_command::~load_percept_command() {}

std::string load_percept_command::description() {
    return std::string("Loads a visual percept from the active file.");
}

bool load_percept_command::update_sub() {
    vsm->load();
    set_status("success");
    return true;
}

command* _make_load_percept_command_(svs_state* state, Symbol* root)
{
    return new load_percept_command(state, root);
}

command_table_entry* load_percept_command_entry()
{
    command_table_entry* e = new command_table_entry();
    e->name = "load-percept";
    e->description = "Load an image from the target file into vision.";
    e->create = &_make_load_percept_command_;
    return e;
}
#endif