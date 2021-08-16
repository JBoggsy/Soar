#ifndef VISUAL_WORKING_MEMORY_H
#define VISUAL_WORKING_MEMORY_H

//////////////
// PREAMBLE //
//////////////
// standard library
///////////////////
#include <map>
#include <string>
// OpenCV
/////////
#include <opencv2/opencv.hpp>

// SVS includes
///////////////
#include "cliproxy.h"
#include "visual_wme.h"

// forward declarations
///////////////////////
class svs;
class soar_interface;
class opencv_image;
template<typename img_t> class visual_archetype;


//////////////////////////
// vwme_metadata STRUCT //
//////////////////////////
/**
 * @brief A struct to hold metadata about the position, rotation, and mirroring
 * of a vwme in visual working memory.
 * 
 * @details x and y are pixel coordinates on an infinite grid
 * @details rotation is the rotation of the vwme in degrees, positive is 
 *          clockwise
 * @details h_ and v_mirror indicate whether the vwme should be mirrored
 *          across the x- or y-axis respectively.
 */
typedef struct vwme_metadata {
    int x, y;
    double rotation;
    bool h_mirror, v_mirror;
} vwme_metadata;


/////////////////////////////////
// visual_working_memory CLASS //
/////////////////////////////////
/**
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
 */
class visual_working_memory : public cliproxy
{
private:
    const static std::string ROS_TOPIC_NAME;
    
    svs* _svs_ptr;
    soar_interface* si;
    Symbol* vwm_link;

    std::map<std::string, image_vwme> vwmes;
    std::map<std::string, vwme_metadata> metadata;

    void _add_vwme(image_vwme new_vwme);
    void _update();

    cv::Mat canvas;
    cv::Point2i origin;
    bool dirty;
    void _generate_canvas();
    void _draw_vwme(image_vwme vwme, vwme_metadata mdata);
    void _draw_canvas();

    /**
     * @brief Saves the entire VWM to a file specified in the command.
     * 
     * @param args The args sent to the command. Should include a file path.
     * @param os The output stream to write to.
     */
    void cli_save(const std::vector<std::string>& args, std::ostream& os);

    /**
     * @brief Retrieves the specified archetype from visual memory and creates
     * a new VWME with the result.
     * 
     * @param args The args sent to the command. Must include an archetype ID to
     * retrieve.
     * @param os The output stream to write to.
     */
    void cli_recall(const std::vector<std::string>& args, std::ostream& os);

public:
    visual_working_memory(svs* svsp, soar_interface* _si, Symbol* link);
    ~visual_working_memory() {}

    visual_working_memory* clone(Symbol* link);

    void add_vwme(image_vwme new_vwme);
    void add_image(opencv_image* new_image, std::string id);
    // void add_varch(visual_archetype* new_varch);

    void remove_vwme(image_vwme target);
    void remove_vwme(std::string target_id);

    /**
     * @brief Translate the vwme with the given vwme ID by the amounts
     * given. 
     * @param d_x The number of pixels to shift the vwme horizontally
     * @param d_y The number of pixels to shift the vwme vertically
     */
    void translate_vwme(std::string vwme_ID, int d_x, int d_y);

    /**
     * @brief Move the vwme with the given ID to the specified coordinates.
     */
    void move_vwme(std::string vwme_ID, int new_x, int new_y);

    /**
     * @brief Rotate the vwme with the given ID by the specified number of 
     * radians. Positive is clockwise.
     */
    void rotate_vwme_rad(std::string vwme_ID, double rads);
    /**
     * @brief Rotate the vwme with the given ID by the specified number of 
     * degrees. Positive is clockwise.
     */
    void rotate_vwme_deg(std::string vwme_ID, double degs);

    /**
     * @brief Flip (i.e., mirror) the vwme with the given ID across the x-axis.
     * This does NOT shift the image.
     */
    void flip_vwme_horiz(std::string vwme_ID);
    /**
     * @brief Flip (i.e., mirror) the vwme with the given ID across the y-axis.
     * This does NOT shift the image.
     */
    void flip_vwme_vert(std::string vwme_ID);

    /**
     * @brief Return the percept represented by the current state of VWM.
     */
    opencv_image* get_percept();

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
};

#endif