#ifndef VISION_INTERFACE_H
#define VISION_INTERFACE_H
/**
 * @file vision_interface.h
 * @author James Boggs
 * @brief Provides a `cliproxy` sub-class for manipulating visual input. In the
 * future this may be extended to providing programmatic ways of getting visual
 * input directly from a source, e.g., a webcam. 
 * 
 * @details 
 * CLI USAGE:
 * 
 * `svs vision` - Prints the last file uploaded to the agent, then this help
 *      text.
 * 
 * `svs vision.setfile <FILEPATH>` - Sets the image upload target to the given
 *      filepath.
 * 
 * `svs vision.load` - Loaads the current image upload target into the agent's 
 *      vision.
 * 
 * `svs vision.save <FILEPATH>` - Saves the current state of the agent's vision
 *      to the specified path.
 * 
 * `svs vision.inject <IMGDATA>` - Injects an image into the agent's visual 
 *      sensory memory. The IMGDATA field should be a base64-encoded image.
 * 
 * `svs vision.remember <ID>` - Adds the current visual input to visual memory.
 * 
 * `svs vision.recall <ID>` - Retrieves the specified archetype from visual 
 *      memory and sets the visual input to the result.
 * 
 * `svs vision.rotate <ANGLE>` - Rotates the current percept by 90, 180, or 270
 *      degrees clockwise.
 * 
 * `svs vision.match` - Match the current percept to the best-fitting percept
 *      stored in visual memory and return the ID of that match. 
 * 
 * `svs vision.export_imagination <FILEPATH` - Like `vision.save`, but for the
 *      imagination buffer.
 * 
 * @version 0.1
 * @date 2020-10-19
 * 
 * @copyright Copyright (c) 2020
 * 
 */

// C++ standard libraries
#include <string>
#include <map>
// SVS inlcudes
#include "cliproxy.h"
// Forward declarations
class svs;
struct vmem_match;


/**
 * @brief The interface class which enables CLI commands which manipulate an
 * agent's vision.
 * 
 */
class vision_interface : public cliproxy {
public:
    vision_interface(svs* svs_ptr);
   ~vision_interface();

    //////////////
    // METHODS //
    ////////////
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
     * @brief Provides the "base" functionality for the `svs vision` command.
     * In particular, it writes the current target image file and a help message.
     * 
     * @param args The args sent to the command. These are discarded.
     * @param os The output stream to write to.
     */
    void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);

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

    /**
     * @brief Adds the current visual input to visual memory.
     * 
     * @param ID The string ID of the visual archetype the current visual 
     * input should be stored as.
     */
    void remember(std::string ID);

    /**
     * @brief Retrieves the specified archetype from visual memory and sets the
     * visual input to the result.
     * 
     * @param ID The string ID of the archetype to retrieve.
     */
    void recall(std::string ID);
    
    /**
     * @brief Match the current percept to the best-fitting percept stored in
     * visual memory and return the ID of that match. 
     * 
     * @param output A `vmem_match` pointer for the function to write the best
     * match to.
     */
    void match(vmem_match* output);

    /**
     * @brief Rotate the current percept by 90, 180, or 270 degrees in the 
     * clockwise direction.
     * 
     * @param amount The amount of rotation in degrees. Must be one of 90, 180,
     * or 270.
     */
    void rotate(int amount);

    bool _file_exists(std::string filepath);
private:
    /////////////////
    // ATTRIBUTES //
    ///////////////
    svs* _svs_ptr;
    std::string _target_filepath;
    
    //////////////
    // METHODS //
    ////////////
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

    /**
     * @brief Adds the current visual input to visual memory.
     * 
     * @param args The args sent to the command. Must include an ID string for
     * the archetype.
     * @param os The output stream to write to.
     */
    void cli_remember(const std::vector<std::string>& args, std::ostream& os);

    /**
     * @brief Retrieves the specified archetype from visual memory and sets the
     * visual input to the result.
     * 
     * @param args The args sent to the command. Must include an archetype ID to
     * retrieve.
     * @param os The output stream to write to.
     */
    void cli_recall(const std::vector<std::string>& args, std::ostream& os);
    
    /**
     * @brief Match the current percept to the best-fitting percept stored in
     * visual memory and return the ID of that match. 
     * 
     * @param args The args sent to the command. None are expected.
     * @param os The output stream to write to. Will write the ID and confidence
     * of the match.
     */
    void cli_match(const std::vector<std::string>& args, std::ostream& os);

    /**
     * @brief Rotate the current percept by 90, 180, or 270 degrees in the 
     * clockwise direction.
     * 
     * @param args The args sent to the command. One of "90", "180", or "270" is
     * expected.
     * @param os The output stream to write to.
     */
    void cli_rotate(const std::vector<std::string>& args, std::ostream& os);

    /**
     * @brief Export the current state of imagination to the given file.
     */
    void cli_export_imagination(const std::vector<std::string>& args, std::ostream& os);
};
#endif