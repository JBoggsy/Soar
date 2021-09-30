#ifndef VISUAL_OPERATIONS_H
#define VISUAL_OPERATIONS_H
#include <unordered_map>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "visual_operation_data_structs.h"

class opencv_image;

/**
 * The `data_dict` type provides string-to-anything mapping for giving/getting
 * data from visual operations.
 */
typedef std::unordered_map<std::string, void*> data_dict;

// Define argument names as strings
#define ARG_TARGET      std::string("target")
#define ARG_FILEPATH    std::string("filepath")
#define ARG_KSIZE       std::string("ksize")
#define ARG_ANCHOR      std::string("anchor")
#define ARG_BORDERTYPE  std::string("borderType")
#define ARG_SIGMAX      std::string("sigmaX")
#define ARG_SIGMAY      std::string("sigmaY")
#define ARG_THRESH      std::string("thresh")
#define ARG_MAXVAL      std::string("maxval")
#define ARG_TYPE        std::string("type")
#define ARG_METHOD      std::string("method")
#define ARG_TEMPLATE    std::string("template")
#define ARG_MINVAL      std::string("minval")
#define ARG_MAXLOC      std::string("maxloc")
#define ARG_MINLOC      std::string("minloc")
#define ARG_BUFFERINDEX std::string("bufferIndex")
#define ARG_VSM         std::string("vsm")
#define ARG_X           std::string("x")
#define ARG_Y           std::string("y")
#define ARG_WIDTH       std::string("width")
#define ARG_HEIGHT      std::string("height")
#define ARG_WINDOWNAME  std::string("windowName")

/**
 * All visual operations are performed IN-PLACE. That is, the target image is
 * altered by the operation. When passing in `data_dict` based arguments, 
 * arguments which have default values can be given a NULL value, but ALL 
 * arguments must be passed in.
 */
namespace visual_ops
{
    /**
     * @brief Load an image from a file
     * @param args
     *        `std::string filepath`: The absolute path to the target file
     *        `opencv_image* target` Object to load the specified file into
     */ 
    void load_from_file(data_dict args);

    /**
     * @brief Save an image to a file
     * @param args
     *        `std::string filepath`: The absolute path to save location
     *        `opencv_image* target` Object to save into the specified file
     */ 
    void save_to_file(data_dict args);

    /**
     * @brief Returns the input image without changes, used to add a no-op node.
     * @param args
     *        `opencv_image* target`: Image to do nothing with
     */
    void identity(data_dict args);

    /**
     * @brief Blur the single target image
     * @param args
     *        `cv::Size ksize`: Blurring kernel size
     *        `cv::Point anchor`: Anchor point; default value Point(-1,-1) means that the anchor is at the kernel center
     *        `int borderType`: Border mode used to extrapolate pixels outside of the image
     *        `opencv_image* target`: The image to blur
     */
    void blur(data_dict args);

    /**
     * @brief Blur the single target image using a Gaussian filter
     * @param args
     *        `cv::Size ksize`: Gaussian kernel size. ksize.width and ksize.height can differ but they both must be positive and odd. Or, they can be zero's and then they are computed from sigma
     *        `double sigmaX`: Gaussian kernel standard deviation in X direction. 
     *        `double sigmaY`: Gaussian kernel standard deviation in Y direction; if sigmaY is zero, it is set to be equal to sigmaX, if both sigmas are zeros, they are computed from ksize.width and ksize.height, respectively (see getGaussianKernel for details); to fully control the result regardless of possible future modifications of all this semantics, it is recommended to specify all of ksize, sigmaX, and sigmaY.
     *        `int borderType`: Border mode used to extrapolate pixels outside of the image
     *        `opencv_image* target`: The image to blur
     */
    void GaussianBlur(data_dict args);

    /**
     * @brief Converts the target image into greyscale.
     * @param args
     *        `opencv_image* target`: The image to convert to greyscale
     */
    void greyscale(data_dict args);

    /** 
     * @brief Binarize a grayscale image according to a threshold
     * @param args
     *        `double thresh`: The threshold value.
     *        `double maxval`: maximum value to use with the THRESH_BINARY and THRESH_BINARY_INV thresholding types.
     *        `int type`: The thresholding type.
     *        `opencv_image* target`: The image to greyscale
     */
    void threshold(data_dict args);

    /**
     * @brief Match a template image and return a new, single-channel image of
     *        comparison results. The returned image will be a single-channel
     *        32-bit floating-point image. If the target image is W×H and template
     *        is w×h , then result is (W−w+1)×(H−h+1).
     * @param args
     *        `int method`: Parameter specifying the comparison method
     *        `opencv_image* target`: The image to be searched for template matches.
     *        `opencv_image* template`: The template image to search for.
     */
    void match_template(data_dict args);

    /**
     * @brief Finds the global minimum and maximum in an array. 
     * @param args Map of non-image arguments to method:
     *        `double* minval`: pointer to the returned minimum value
     *        `double* maxval`: pointer to the returned maximum value
     *        `cv::Point* minloc`: pointer to the returned minimum location
     *        `cv::Point* maxloc`: pointer to the returned maximum location
     *        `opencv_image* target`: The single-channel input image to find
     *                                min/max of
     */
    void min_max_loc(data_dict args);

} // namespace visual_ops

#endif