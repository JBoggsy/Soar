#ifndef VISUAL_WORKING_MEMORY_H
#define VISUAL_WORKING_MEMORY_H

//////////////
// PREAMBLE //
//////////////
// standard lib includes
////////////////////////
#include <string>
#include <vector>
// SVS includes
///////////////
#include "soar_interface.h"
#include "image.h"
#include "cliproxy.h"
#include "visual_operation_graph.h"
#include "visual_input_buffer.h"
// forward definitions
//////////////////////
class svs;


/////////////////////////////////
// visual_working_memory CLASS //
/////////////////////////////////
class visual_working_memory: public cliproxy
{
private:
    typedef std::unordered_map<int, visual_operation_node*> id_node_map;
    const static std::string ROS_TOPIC_NAME;

    svs*            svs_ptr;
    soar_interface* si;
    Symbol*         vwm_link;

    int                 num_operations;
    Symbol*             num_operations_symbol;
    int                 next_vop_node_id;
    std::vector<int>    source_vop_node_ids;
    id_node_map         vop_nodes;

public:
    visual_working_memory(svs* svs_ptr, soar_interface* si, Symbol* vwm_link);
    ~visual_working_memory();
    visual_working_memory* clone(Symbol* vwm_link);

    //////////////////////////////
    // VISUAL OPERATION METHODS //
    //////////////////////////////
    int add_visual_operation(std::string op_name, data_dict op_args);
    bool edit_visual_operation(int node_id, data_dict new_op_args);
    int remove_visual_operation(int node_id);


    //////////////////////
    // CLIPROXY METHODS //
    //////////////////////
    /**
     * @brief Provide a map of the sub-commands for this command to the CLI
     * parser.
     *
     * @details Todo
     *
     * @todo Write details
     *
     * @param c A mapping of string identifiers to `cliproxy` instance.
     */
    void proxy_get_children(std::map<std::string, cliproxy*>& c);
    /**
     * @brief TODO
     *
     * @todo Fill in brief and details docs
     *
     * @param args The args sent to the command. These are discarded.
     * @param os The output stream to write to.
     */
    void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);
};

#endif
