/**
 * NOTE TO THE FUTURE
 * ------------------
 * Inevitably I, or someone else, will need to add a new SVS command for agents
 * to use. Since rediscovering how Soar handles SVS commands involves navigating
 * the maze-like codebase, I've documented the process here, with an eye towards
 * how to add new SVS commands.
 *
 * # Executive Summary:
 * --------------------
 * 1. Create a new `<command_name>.cpp` file in the `commands` directory. **All
 *    current commands are defined in this way, so feel free to use them as a
 *    template.*
 *
 * 2. In that file, define a new class named `<command_name>_command` that
 *    inherits from `command` and implement the necessary virtual functions:
 *
 *   - `update_sub()`: **This is where the command's logic goes.** This function
 *     is called during the input phase by the SVS state object that owns the
 *     command. It should return `true` if the command has completed, and
 *     `false` if it needs to be called again during the next input phase.
 *
 *   - constructor: Takes a `svs_state*` and a `Symbol*` as arguments.
 *
 *   - `description()`: Returns a string describing the command. This is used
 *     somewhere in the CLI code, IIRC, and isn't strictly necessary.
 *
 *  - `command_type()`: Returns either `SVS_READ_COMMAND` or
 *    `SVS_WRITE_COMMAND`.
 *
 * 3. In the same file, define a function named `_make_<command_name>_command_`
 *    which takes a `svs_state*` and a `Symbol*` as arguments and returns a new
 *    instance of the `<command_name>_command` class. This function is used as
 *    the `create` field of the `command_table_entry` object, which is in turn
 *    stored in the `command_table` object. This allows the SVS command
 *    processing code to look up the command in the command table and create a new
 *    instance of the command based on the command's WME.
 *
 * 4. Define a function named `<command_name>_command_entry` that creates a new
 *    `command_table_entry` object, sets its `name` and `description` fields,
 *    and sets its `create` field to the `_make_<command_name>_command_`
 *    function, then returns the new `command_table_entry` object. This function
 *    is used to add the new command to the command table, as seen below.
 *
 * 5. In the "COMMAND TABLE ENTRIES" section in the code below, add a new line
 *    following the existing code that creates a new `command_table_entry`
 *    object for you new command by calling the `<command_name>_command_entry`
 *    function.
 *
 * 6. In the "COMMAND TABLE INIT" section in the code below, add a new line
 *    following the existing code that adds the new `command_table_entry` object
 *    to the command table by calling the `add` method on the `command_table`.
 *
 *
 * # SVS Command Processing System
 * -------------------------------
 * 1. During the Soar run loop (see `do_one_top_level_phase` at
 *    `run_soar.cpp:427`), Soar calls `svs::input_callback` and
 *    `svs::output_callback` at the beginning of the input and output phases,
 *    respectively.
 *
 * 2. `svs::output_callback` is where new SVS commands are ingested, but NOT
 *    where they are pased. The callback loops through each SVS state and calls
 *    `svs_state::process_cmds`, which looks at each of the child WMEs of the
 *    ^svs.command ID WME, turns them into commands, and diffs this list of
 *    commands with the current list of commands (`svs_state::curr_cmds`) for
 *    the SVS state. Any new commands are added to the list of active commands,
 *    while any old commands missing from the new list are removed. **At no
 *    point in this process are the commands actually executed.**
 *
 *      a. New commands are created and added to the list of active commands.
 *         Commands are created by calling `command_table::make_command`
 *         (defined below) with the command's WME structure. This function looks
 *         up the command in the command table and calls the resulting
 *         `command_table_entry`'s `create` method to create a new instance of
 *         the `<command_name>_command` class based on the SVS state and command
 *         WME.
 *
 * 3. The input phase callback is where the new commands are processed. The
 *    callback loops through each SVS state and calls `svs_state::update_cmd_results`
 *    once for each type of SVS command (read and write). This function loops
 *    through each active command in the SVS state (`svs_state::curr_cmds`) and
 *    calls the command's `update` method, which in turn calls its `update_sub`
 *    method. This is where the command's logic is executed.
 */


#include <stdlib.h>
#include <ctype.h>
#include <sstream>
#include <limits>
#include <iomanip>

#include "command.h"
#include "command_table.h"
#include "svs.h"
#include "scene.h"
#include "soar_interface.h"
#include "symbol.h"


command_table& get_command_table()
{
    static command_table inst;
    return inst;
}

////////////////////////////
// COMMAND TABLE ENTRIES //
//////////////////////////

// SPATIAL SCENE COMMANDS
/////////////////////////

command_table_entry* extract_command_entry();
command_table_entry* extract_once_command_entry();

command_table_entry* add_node_command_entry();
command_table_entry* copy_node_command_entry();
command_table_entry* set_transform_command_entry();
command_table_entry* copy_transform_command_entry();
command_table_entry* delete_node_command_entry();

command_table_entry* set_tag_command_entry();
command_table_entry* delete_tag_command_entry();

// VISUAL WORKING MEMORY COMMANDS
/////////////////////////////////
command_table_entry* add_vop_node_command_entry();
command_table_entry* edit_vop_node_command_entry();
command_table_entry* learn_percept_command_entry();

////////////////////////
// COMMAND TABLE INIT //
////////////////////////
command_table::command_table()
{
    set_help("Prints out a list of all soar commands");

    // Scene graph
    add(extract_command_entry());
    add(extract_once_command_entry());
    add(add_node_command_entry());
    add(copy_node_command_entry());
    add(set_transform_command_entry());
	add(copy_transform_command_entry());
    add(delete_node_command_entry());
    add(set_tag_command_entry());
    add(delete_tag_command_entry());

    // Visual Working Memory
    add(add_vop_node_command_entry());
    add(edit_vop_node_command_entry());
    add(learn_percept_command_entry());
}

command* command_table::make_command(svs_state* state, wme* w)
{
    std::string name;
    Symbol* id;
    soar_interface* si;

    si = state->get_svs()->get_soar_interface();
    if (!get_symbol_value(si->get_wme_attr(w), name))
    {
        return NULL;
    }
    if (!si->get_wme_val(w)->is_sti())
    {
        return NULL;
    }
    id = si->get_wme_val(w);

    std::map<std::string, command_table_entry*>::iterator i = table.find(name);
    if (i != table.end())
    {
        return i->second->create(state, id);
    }
    else
    {
        return NULL;
    }
}

void command_table::add(command_table_entry* e)
{
    table[e->name] = e;
}

void command_table::proxy_get_children(std::map<std::string, cliproxy*>& c)
{
    std::map<std::string, command_table_entry*>::iterator i, iend;
    for (i = table.begin(), iend = table.end(); i != iend; ++i)
    {
        c[i->first] = i->second;
    }
}

void command_table::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os)
{
    os << "====================== COMMAND TABLE =======================" << std::endl;
    std::map<std::string, command_table_entry*>::iterator i;
    for (i = table.begin(); i != table.end(); i++)
    {
        os << "  " << std::setw(22) << std::left << i->first << " | " << i->second->description << std::endl;
    }
    os << "===========================================================" << std::endl;
    os << "For specific command info, use the command 'svs commands.command_name'" << std::endl;
}

command_table_entry::command_table_entry()
    : create(NULL), description("")
{
    set_help("Reports information about this command");
}

void command_table_entry::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os)
{
    os << "Command: " << name << std::endl;
    os << "  " << description << std::endl;
    os << "  Parameters:" << std::endl;
    std::map<std::string, std::string>::iterator i;
    for (i = parameters.begin(); i != parameters.end(); i++)
    {
        os << "    " << std::setw(15) << std::left << i->first << " | " << i->second << std::endl;
    }
}
