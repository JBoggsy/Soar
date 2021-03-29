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
#include "image.h"
#include "cliproxy.h"

// forward definitions
//////////////////////
class svs;


/////////////////////////////////
// visual_sensory_memory CLASS //
/////////////////////////////////
class visual_sensory_memory: public cliproxy
{
private:
    const static std::string ROS_TOPIC_NAME;
    const static int PERCEPT_BUFFER_SIZE = 1;

    svs* svs_ptr;
    soar_interface* si;
    opencv_image* percept_buffer [PERCEPT_BUFFER_SIZE];
    Symbol* vsm_link;
    wme* updated_link;
    int update_counter;
    
    std::string _target_filepath;
    bool _file_exists(std::string filepath);
public:
    visual_sensory_memory(svs* svs_ptr, soar_interface* _si);
    ~visual_sensory_memory();

    void add_wm_link(Symbol* vsm_link);

    void update_percept_buffer(const cv::Mat& new_percept);
    void draw_percept_buffer();

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