#ifndef VISUAL_OPERATION_GRAPH_H
#define VISUAL_OPERATION_GRAPH_H

//C++ STDLIB includes
#include <vector>
#include <unordered_map>
#include <unordered_set>
// SVS includes
#include "cliproxy.h"
#include "visual_operation_data_structs.h"
#include "visual_operation.h"
#include "image.h"
class visual_sensory_memory;
class visual_operation_graph;


class visual_operation_node {
private:
    static int NEXT_NODE_ID_;

    int                                     id_;
    std::vector<int>                        child_ids_;
    std::unordered_map<std::string, int>    parent_ids_;

    std::string                     op_name_;
    data_dict                       parameters_;
    void                            (*operation_)(data_dict args);  
    visual_ops::vop_params_metadata params_metadata_;

    visual_operation_graph* vog_;
    Symbol*                 node_link_;
public:
    visual_operation_node(std::string op_name, data_dict args, soar_interface* si, Symbol* node_link);
    ~visual_operation_node();
    
    int get_id() { return id_; }
    std::vector<int>* get_child_ids() { return &child_ids_; }
    std::unordered_map<std::string, int>* get_parent_ids() { return &parent_ids_; }
    bool evaluate();
};


typedef std::unordered_map<int, visual_operation_node*>          id_node_map;

/**
 * @brief Class which maintains a feed-forward graph structure of visual operations
 * to be performed on visual input.
 * @details Each Soar agent with SVS enabled has a visual operations graph
 * attached to its visual sensory memory. By default the graph is empty, but the
 * agent can attach one or more visual operations to the graph, specifying input
 * source, output type, and the operation performed. These individual operations
 * are organized in a feed-forward graph structure wherein the output of a parent
 * node is fed as the input to its children. This allows the agent to construct
 * complex visual filters with high granularity.
 * 
 * Operation inputs can be images or structured data, and the can output the same.
 * In addition to a child operation, operations can output to visual or symbolic
 * working memory. Operations can receive the contents of visual sensory memory
 * as input instead of other operation nodes.
 * 
 * The `visual_operations_graph` contains an array of `visual_operation_node`
 * structs, which is resized when needed by the `insert()` method. The 
 * `visual_operation_node` structs keep track of their parents and children to
 * allow graph traversal. The `visual_operations_graph` also maintains an array
 *  of `image` objects (e.g., `opencv_iamge` or `pcl_image`) which are given to
 * the `visual_operation_nodes`s. 
 * 
 * When the `visual_operation_graph.evaluate()` method is called, the graph is
 * traversed recursively upwards from the leaf nodes. Each node requires one or
 * more input images, which are obtained from parent nodes. Each parent node has
 * a corresponding identifier which indicates its argument. Since all parent nodes
 * need to be evaluated before the child node can be evaluated, parent nodes are
 * recursively evaluated in order to get the input images for the child. Once
 * all parents are evaluated, the child performs its own operation.
 */
class visual_operation_graph : public cliproxy {
private:
    visual_sensory_memory* vsm_;
    soar_interface* si_;
    int num_operations_ = 0;
    visual_operation_node* root_node_;
    id_node_map nodes_;
    std::unordered_set<int> leaf_nodes_;

    // Node images are created using `new` by the node when it is evaluated
    // the first time, and are all `delete`d at the end of evaluation.
    int_image_map node_images_;

    /**
     * @brief Evaluate the specified node on the visual operation graph. If
     * the node's parents have not been evaluated yet, they will be evaluated
     * first. 
     */
    void evaluate_node(visual_operation_node* node);
public:
    visual_operation_graph(visual_sensory_memory* vsm);
    ~visual_operation_graph();

    int get_num_operations() { return num_operations_; }

    /**
     * @brief Insert a new operation into the graph as a child of the nodes
     * with the specified ids. 
     * @param `parents`: A map from argument names to node ids indicating the
     *        correspondence between the image-type arguments of the visual
     *        operation and the parents of the node. If this is empty, the node
     *        will be a new source node.
     * @param `params`: A newly allocated `data_dict` containing the 
     *        parameters for the visual operation
     * 
     * @returns The ID of the new visual operations in the graph. If insertion
     * fails, will return -1.
     */
    int insert(std::unordered_map<std::string, int> parents, data_dict* params, void (*operation)(data_dict data_in));

    /**
     * @brief Remove the operation with the specified id. 
     * ALL CHILDREN ARE DESTROYED. The root node cannot be removed.
     * 
     * @returns The new number of visual operations in the graph. If removal
     * fails, the number will be the same as it was prior to the operation.
     */
    int remove(int target_id);

    /**
     * @brief Evaluate every node in the graph once. Nodes are evaluated
     * recursively upwards, starting with the leaf nodes. In the process of
     * evaluating a node, its parent nodes are evaluated in order to generate
     * the image inputs for the node itself. Nodes are evaluated at most
     * once and the result of that evaluation is cached, to be accessed by
     * children of the node. Once every child of a node has been evaluated,
     * its image is destroyed to save memory.
     */
    void evaluate();

    //////////////////////
    // CLIPROXY METHODS //
    //////////////////////
    /**
     * @brief Provide a map of the sub-commands for this command to the CLI 
     * parser.
     * 
     * @details The `svs vision` command family is outlined at the top of this
     * file.
     * 
     * @param c A mapping of string identifiers to `cliproxy` instance.
     */
    void proxy_get_children(std::map<std::string, cliproxy*>& c);
    /**
     * @brief Provides the "base" functionality for the `svs vsm` command.
     * In particular, it writes the current target image file and a help message.
     * 
     * @param args The args sent to the command. These are discarded.
     * @param os The output stream to write to.
     */
    void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);
};
#endif