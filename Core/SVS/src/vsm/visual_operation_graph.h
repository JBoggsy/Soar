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
    int id_;
    std::unordered_set<int> child_ids_;
    std::unordered_map<std::string, int> parent_ids_;

    std::string op_name_;
    data_dict parameters_;
    void (*operation_)(data_dict args);  
    visual_ops::vop_params_metadata op_metadata_;

    visual_operation_graph* vog_;
    opencv_image* node_image_;

    soar_interface* si_;
    Symbol* node_link_;
    Symbol* op_name_sym_;
    Symbol* node_id_sym_;
    std::unordered_map<std::string, Symbol*> param_syms_;
public:
    visual_operation_node(std::string op_name, visual_ops::vop_params_metadata op_metadata, data_dict* params, 
                          visual_operation_graph* vog, soar_interface* si, Symbol* node_link);
    ~visual_operation_node();
    
    void set_id(int id) { id_ = id; }
    int get_id() { return id_; }
    std::unordered_set<int>* get_child_ids() { return &child_ids_; }
    std::unordered_map<std::string, int>* get_parent_ids() { return &parent_ids_; }

    void add_child_id(int id) { child_ids_.insert(id); }
    void remove_child_id(int id) {child_ids_.erase(id); }

    bool evaluate();
    opencv_image* get_node_image();
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
 * The `visual_operations_graph` contains a mapping from integer node IDs to 
 * `visual_operation_node` objects, The `visual_operation_node`s keep track of 
 * their parents and children to allow graph traversal. They also hold their
 * own parameters, a pointer to the correct visual operation method, and the
 * results of their operation.
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

    int num_operations_;
    int next_node_id_;
    id_node_map nodes_;
    std::unordered_set<int> leaf_nodes_;
    std::unordered_set<int> evaluated_nodes_;

    Symbol* vog_link_;
    Symbol* num_ops_sym_;
    std::vector<Symbol*> node_links_;

    /**
     * @brief Evaluate the specified node on the visual operation graph. If
     * the node's parents have not been evaluated yet, they will be evaluated
     * first. 
     */
    void evaluate_node(visual_operation_node* node);
public:
    visual_operation_graph(visual_sensory_memory* vsm, soar_interface* si, Symbol* vsm_link);
    ~visual_operation_graph();

    int get_num_operations() { return num_operations_; }

    int assign_new_node_id() {return next_node_id_++;}

    /**
     * @brief Insert a new operation into the graph as a child of the nodes
     * with the specified ids. 
     * @param `op_name`: The name of the operation of the new node. This should
     *        correspond with a valid key in `visual_ops::vops_param_table`.
     * @param `params`: A newly allocated `data_dict` containing the 
     *        parameters for the visual operation
     * 
     * @returns The ID of the new visual operations in the graph. If insertion
     * fails, will return -1.
     */
    int insert(std::string op_name, data_dict* params, std::unordered_map<std::string, int> parent_ids);

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
    
    /**
     * @brief 
     * 
     * @param child_id 
     * @param parent_id 
     * @return true The child node id was successfully added to the parent node.
     * @return false The child node id couldn't be added to the parent node.
     */
    bool add_child_to_node(int child_id, int parent_id);

    /**
     * @brief Designate the node with the specified ID as a leaf node(i.e.,
     * a node with no other nodes using it as a `target`.) This method does
     * NOT check that the given node is, in fact, a leaf node.
     * 
     * @param node_id The node id of the node to be designated a leaf node.
     */
    void add_leaf_node(int node_id) { leaf_nodes_.insert(node_id); }

    /**
     * @brief Undesignate the node with the specified ID as a leaf node(i.e.,
     * a node with no other nodes using it as a `target`.) This method does
     * NOT check that the given node is not, in fact, a leaf node.
     * 
     * @param node_id The node id of the node to be undesignated as a leaf node.
     */
    void remove_leaf_node(int node_id) { leaf_nodes_.erase(node_id); }

    /**
     * @brief Marks the node with the given id as evaluated for the current
     * evaluation run.
     * 
     */
    void mark_node_evaluated(int node_id) { evaluated_nodes_.insert(node_id); }

    /**
     * @brief Get the node image of the given node.
     * 
     * For now, assume a node ALWAYS has precisely 1 node image.
     * 
     * @param node_id 
     * @return opencv_image* The image of the given node, or NULL if such a
     *         node doesn't exist or cannot be evaluated.
     */
    opencv_image* get_node_image(int node_id);


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