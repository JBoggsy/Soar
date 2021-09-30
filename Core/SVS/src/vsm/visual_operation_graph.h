#ifndef VISUAL_OPERATION_GRAPH_H
#define VISUAL_OPERATION_GRAPH_H

//C++ STDLIB includes
#include <vector>
#include <unordered_map>
#include <unordered_set>
// SVS includes
#include "visual_operation_data_structs.h"
#include "visual_operation.h"
#include "image.h"
class visual_sensory_memory;


typedef struct visual_operation_node {
    int id;
    std::unordered_set<int> children;
    std::unordered_map<std::string, int> parents;
    data_dict parameters;
    void (*v_op)(data_dict args);
} visual_operation_node;

// DEPRECATED typedef std::unordered_set<visual_operation_node*>              node_ptr_set;
// DEPRECATED typedef std::unordered_map<std::string, visual_operation_node*> str_node_ptr_map;
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
class visual_operation_graph {
    public:
        visual_operation_graph(visual_sensory_memory* _vsm);
        ~visual_operation_graph();

        int get_num_operations() { return num_operations; }

        /**
         * @brief Insert a new operation into the graph as a child of the nodes
         * with the specified ids. The root node always exists and has id=0. 
         * @param `parents`: A map from argument names to node ids indicating the
         *        correspondence between the image-type arguments of the visual
         *        operation and the parents of the node
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

    private:
        visual_sensory_memory* vsm;
        int num_operations = 0;
        int next_op_id = 0;
        visual_operation_node* root_node;
        id_node_map nodes;
        std::unordered_set<int> leaf_nodes;

        // Node images are created using `new` by the node when it is evaluated
        // the first time, and are all `delete`d at the end of evaluation.
        int_image_map node_images;

        /**
         * @brief Evaluate the specified node on the visual operation graph. If
         * the node's parents have not been evaluated yet, they will be evaluated
         * first. 
         */
        void evaluate_node(visual_operation_node* node);
};
#endif