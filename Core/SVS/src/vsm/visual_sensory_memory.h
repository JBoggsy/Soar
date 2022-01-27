#ifndef VISUAL_SENSORY_MEMORY_H
#define VISUAL_SENSORY_MEMORY_H

//////////////
// PREAMBLE //
//////////////
// standard lib includes
////////////////////////
#include <string>
// SVS includes
///////////////
#include "soar_interface.h"
#include "image.h"
#include "cliproxy.h"
#include "visual_operation_graph.h"
#include "visual_buffer.h"
// forward definitions
//////////////////////
class svs;


/////////////////////////////////
// visual_sensory_memory CLASS //
/////////////////////////////////
class visual_sensory_memory: public cliproxy
{
private:
    const static std::string ROS_TOPIC_NAME_;

    svs* svs_ptr_;
    soar_interface* si_;
    Symbol* vsm_link_;
    
    visual_buffer* visual_buffer_;
    visual_operation_graph* vop_graph_;
    
    std::string target_filepath_;

    bool _file_exists(std::string filepath);
public:
    visual_sensory_memory(svs* svs_ptr, soar_interface* si, Symbol* vsm_link);
    ~visual_sensory_memory();
    
    visual_buffer* get_visual_buffer() { return visual_buffer_; }
    int get_visual_buffer_size() { return visual_buffer_->get_size(); }
    opencv_image* get_vision();
    opencv_image* get_vision(int index);
    void update_visual_buffer(const cv::Mat& new_percept);
    void draw_visual_buffer();

    visual_operation_graph* get_vop_graph() { return vop_graph_; }
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

    ////////////////////////
    // CLI-BASED COMMANDS //
    ////////////////////////

    /**
     * @brief Sets the target image file for the load command.
     * 
     * @param args The args sent to the command. Should include a file path.
     * @param os The output stream to write to.
     */
    void cli_setfile(const std::vector<std::string>& args, std::ostream& os);

    /**
     * @brief Loads the target image file into the agent's visual input.
     * 
     * @param args The args sent to the command. These are discarded.
     * @param os The output stream to write to.
     */
    void cli_load(const std::vector<std::string>& args, std::ostream& os);

    /**
     * @brief Saves the agent's visual input to a file specified in the command.
     * 
     * @param args The args sent to the command. Should include a file path.
     * @param os The output stream to write to.
     */
    void cli_save(const std::vector<std::string>& args, std::ostream& os);

    /**
     * @brief Injects an image into the agent's visual sensory memory. The 
     * `imgdata` field should be a base64-encoded image.
     * 
     * @param args The args sent to the command. Must be a base64-encoded image.
     * @param os The output stream to write to.
     */
    void cli_inject(const std::vector<std::string>& args, std::ostream& os);

    ////////////////////////
    // WME-BASED COMMANDS //
    ////////////////////////
    /**
     * @brief Sets the target image file for the load command.
     * 
     * @param filepath The target image file. Must exist.
     */
    void setfile(std::string filepath);

    /**
     * @brief Loads the target image file into the agent's visual input.
     * 
     * @param args The args sent to the command. These are discarded.
     * @param os The output stream to write to.
     */
    void load();

    /**
     * @brief Saves the agent's visual input to a file specified in the command.
     * 
     * @param filepath The path to save the current visual input to.
     */
    void save(std::string filepath);
};

#endif