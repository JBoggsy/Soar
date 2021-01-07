#include "command.h"
#include "command_table.h"

command_table_entry* imagine_flip_percept_command_entry() {
    command_table_entry* e = new command_table_entry();
    e->name = "imagine_flip_percept";
    e->description = "";
    return e;
}