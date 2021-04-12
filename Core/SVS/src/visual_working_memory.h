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
class visual_working_memory
{
private:
    const static std::string ROS_TOPIC_NAME;
    
    svs* svs_ptr;
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

public:
    visual_working_memory(svs* svsp, soar_interface* _si, Symbol* link);
    ~visual_working_memory();

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

    void identify();

    /**
     * @brief Return the percept represented by the current state of VWM.
     * 
     */
    opencv_image* get_percept();
};

#endif