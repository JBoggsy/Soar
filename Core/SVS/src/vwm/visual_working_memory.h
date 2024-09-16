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
#include "visual_operation.h"
// Third-party includes
////////////////////////
#ifdef ENABLE_TORCH
#include <torch/script.h>
#endif
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
    typedef std::unordered_map<std::string, std::vector<int>> str_nodes_map;
    const static std::string ROS_TOPIC_NAME;

    svs*            svs_ptr;
    soar_interface* si;
    Symbol*         vwm_link;

    int                 num_operations;
    Symbol*             num_operations_symbol;
    wme*                num_operations_wme;
    int                 next_vop_node_id;
    id_node_map         vop_nodes;
    str_nodes_map       vib_vop_node_ids;

public:
    visual_working_memory(svs* svs_ptr, soar_interface* si, Symbol* vwm_link);
    ~visual_working_memory();
    visual_working_memory* clone(Symbol* vwm_link);

    //////////////////////////////
    // VISUAL OPERATION METHODS //
    //////////////////////////////
    /**
     * @brief Adds a new visual operation to the vop graph.
     *
     * @todo Add details
     *
     * @param op_type The name of the operation type to add.
     * @param op_args The values for the vop's arguments as a `data_dict`.
     * @param parent_ids The node ids of the parent(s) of the new node
     * @return The node id of the new vop node, or -1 if the method fails.
     */
    int add_visual_operation(std::string op_type, data_dict* op_args, std::unordered_map<std::string, int> parent_ids);

    /**
     * @brief Removes a visual operation from the vop graph.
     *
     * @todo Fill in details
     *
     * @note ALL CHILDREN ARE DESTROYED.
     *
     * @param node_id The node id of the vop to remove.
     * @return The number of remaining vop nodes, or -1 if the method fails.
     */
    int remove_visual_operation(int node_id);

    /**
     * @brief Add the given child node to the given parent node, and ensure the parent
     * node is no longer marked as a leaf node.
     *
     * @param child_id
     * @param parent_id
     * @return true The child node id was successfully added to the parent node.
     * @return false The child node id couldn't be added to the parent node.
     */
    bool add_child_to_node(int child_id, int parent_id);

    /**
     * @brief Evaluates the visual operation graph starting from the given node.
     * This will re-evaluate the entire sub-graph rooted at the given node so
     * that any change to the image resulting from node will be reflected in all
     * descendant nodes. Uses a breadth-first traversal to ensure all image
     * sources for a given node are evaluated before that node is.
     *
     * @param node_id
     * @return True if the evaluation is successful, false otherwise.
     */
    bool evaluate_from_node(int node_id);

    /**
     * @brief Evaluates the vop sub-graphs rooted at each get-from-vib vop which
     * gets a frame from the specified vib.
     *
     * @param vib_id The string id of the vib whose associated get-from-vib vops
     * need to be evaluated.
     * @return True if the evaluation is successful, false otherwise.
     */
    bool evaluate_vib_nodes(std::string vib_id);


    int assign_new_node_id();
    visual_operation_node* get_node(int node_id);

    /**
     * @brief Get the node image of the given node. If the node has already been evaluated,
     * then just get its image. Otherwise, evaluate it first and then get the resulting image.
     * For now, assume a node ALWAYS outputs precisely one node image and it is stored in
     * `parameters_["source"]`. Other images can be used as input, but only the `source` is
     * used as output.
     *
     * @param node_id The id of the node whose image is requested. A node_id of -1 indicates
     *                an "origin" node and will always result in a new, blank image.
     *
     * @return opencv_image* The image of the given node, or NULL if such a
     *         node doesn't exist or cannot be evaluated.
     */
    opencv_image* get_node_image(int node_id);

    /**
     * @brief Generate a DOT language representation of the visual operations graph.
     */
    std::string get_vog_dot_string();


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

    void cli_list_nodes(const std::vector<std::string>& args, std::ostream& os);
    void cli_get_node_info(const std::vector<std::string>& args, std::ostream& os);
    void cli_get_node_image(const std::vector<std::string>& args, std::ostream& os);
    void cli_get_vog_dot(const std::vector<std::string>& args, std::ostream& os);
};

#endif
