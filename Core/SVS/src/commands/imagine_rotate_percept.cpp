#include "command.h"
#include "command_table.h"

command_table_entry* imagine_rotate_percept_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "imagine_rotate_percept";
    e->description = "";
    return e;
}