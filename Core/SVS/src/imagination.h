#ifndef IMAGINATION_H
#define IMAGINATION_H

#include <unordered_map>

/**
 * @brief Abstract base class for `imagination_` classes. `imagination_` classes
 * provide imaginative (i.e., fictive percept) functionality to a Soar agent.
 * An `imagination_` class is needed for each percept type, such as 
 * `imagination_opencv` or `imagination_pcl` classes.
 * 
 * `imagination_` class responsibilities:
 * - provide access to imagined image
 * - maintain list and state of imagined percepts
 * - provide interface for adding, removing, and manipulating imagined percepts
 */
template<typename percept_t>
class imagination_base {
public:
    // IMAGINATION MANIPULATION
    ///////////////////////////
    /**
     * @brief Copy the current state of the imagination to the output percept
     */
    virtual void get_image(percept_t& output) = 0;

    /**
     * @brief "Imagine" a new percept by adding it to the imagination.
     * @returns A unique ID identifying the percept in the imagination.
     */
    virtual int add_percept(percept_t new_percept) = 0;
    
    /**
     * @brief Remove a percept from the imagination using its unique ID.
     */
    virtual void remove_percept(int percept_ID) = 0;

    // UTILITIES
    ////////////

    /**
     * @brief Indicate whether the imagination has been changed since the last
     * time the imagined percept was retrieved.
     * @returns True if the imagination has new changes
     */
    bool changed() { return dirty; } 
protected:
    bool dirty = false;

    int next_percept_id = 0;
    std::unordered_map<int, percept_t> percepts;
};

/////////////////////////
// OPENCV IMAGINATION //
///////////////////////
#ifdef ENABLE_OPENCV
#include "image.h"

/**
 * @brief A struct to hold metadata about the position, rotation, and mirroring
 * of a percept in the imagination.
 * 
 * @details x and y are pixel coordinates on an infinite grid
 * @details rotation is the rotation of the percept in degrees, positive is 
 *          clockwise
 * @details h_ and v_mirror indicate whether the percept should be mirrored
 *          across the x- or y-axis respectively.
 */
typedef struct imagination_opencv_percept_metadata {
    int x, y;
    double rotation;
    bool h_mirror, v_mirror;
} imagination_opencv_percept_metadata;

class imagination_opencv : imagination_base<opencv_image> {
public:
    // INHERITED
    ////////////
    /**
     * @brief Copy the current state of the imagination to the output percept
     */
    void get_image(opencv_image& output);
    /**
     * @brief "Imagine" a new percept by adding it to the imagination.
     * @returns A unique ID identifying the percept in the imagination.
     */
    int add_percept(opencv_image new_percept);
    /**
     * @brief Remove a percept from the imagination using its unique ID.
     */
    void remove_percept(int percept_ID);

    // NOVEL
    ////////
    imagination_opencv();

    /**
     * @brief "Imagine" a new percept by adding it to the imagination in a 
     * specific location. The location is given as a pair of integers (x, y)
     * which designate the location where the center of the percept should go.
     * @returns A unique ID identifying the percept in the imagination.
     */
    int add_percept(opencv_image new_percept, int x, int y);

    /**
     * @brief Translate the percept with the given percept ID by the amounts
     * given. 
     * @param dX The number of pixels to shift the percept horizontally
     * @param dY The number of pixels to shift the percept vertically
     */
    void translate_percept(int percept_ID, int dX, int dY);

    /**
     * @brief Move the percept with the given ID to the specified coordinates.
     */
    void move_percept(int percept_ID, int new_x, int new_y);

    /**
     * @brief Rotate the percept with the given ID by the specified number of 
     * radians. Positive is clockwise.
     */
    void rotate_percept_rad(int percept_ID, double rads);
    /**
     * @brief Rotate the percept with the given ID by the specified number of 
     * degrees. Positive is clockwise.
     */
    void rotate_percept_deg(int percept_ID, double degs);

    /**
     * @brief Flip (i.e., mirror) the percept with the given ID across the x-axis.
     * This does NOT shift the image.
     */
    void flip_percept_horiz(int percept_ID);
    /**
     * @brief Flip (i.e., mirror) the percept with the given ID across the y-axis.
     * This does NOT shift the image.
     */
    void flip_percept_vert(int percept_ID);
private:
    void _generate_canvas();
    void _draw_percept(opencv_image percept, imagination_opencv_percept_metadata mdata);
    void _draw_canvas();

    cv::Mat canvas;
    cv::Point2i origin;
    std::unordered_map<int, imagination_opencv_percept_metadata> metadata;
};
#endif
#endif