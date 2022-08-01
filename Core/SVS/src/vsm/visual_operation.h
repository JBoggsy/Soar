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

// Define operation names as strings
#define VOP_GET_FROM_VSM        std::string("get-from-vsm")
#define VOP_LOAD_FROM_FILE      std::string("load-from-file")
#define VOP_SAVE_TO_FILE        std::string("save-to-file")
#define VOP_DISPLAY_IMAGE       std::string("display-image")
#define VOP_IDENTITY            std::string("identity")
#define VOP_BLUR                std::string("blur")
#define VOP_GAUSSIANBLUR        std::string("gaussian-blur")
#define VOP_GREYSCALE           std::string("greyscale")
#define VOP_THRESHOLD           std::string("threshold")
#define VOP_MATCH_TEMPLATE      std::string("match-template")
#define VOP_CROP_TO_ROI         std::string("crop-to-roi")
#define VOP_MIN_MAX_LOC         std::string("min-max-loc")

// Define argument names as strings
#define VOP_ARG_TARGET      std::string("target")
#define VOP_ARG_FILEPATH    std::string("filepath")
#define VOP_ARG_SIZEX       std::string("size-x")
#define VOP_ARG_SIZEY       std::string("size-y")
#define VOP_ARG_ANCHORX     std::string("anchor-x")
#define VOP_ARG_ANCHORY     std::string("anchor-y")
#define VOP_ARG_BORDERTYPE  std::string("border-type")
#define VOP_ARG_SIGMAX      std::string("sigma-x")
#define VOP_ARG_SIGMAY      std::string("sigma-y")
#define VOP_ARG_THRESH      std::string("thresh")
#define VOP_ARG_MAXVAL      std::string("maxval")
#define VOP_ARG_TYPE        std::string("type")
#define VOP_ARG_METHOD      std::string("method")
#define VOP_ARG_TEMPLATE    std::string("template")
#define VOP_ARG_MINVAL      std::string("minval")
#define VOP_ARG_MAXLOCX     std::string("maxloc-x")
#define VOP_ARG_MAXLOCY     std::string("maxloc-y")
#define VOP_ARG_MINLOCX     std::string("minloc-x")
#define VOP_ARG_MINLOCY     std::string("minloc-y")
#define VOP_ARG_BUFFERINDEX std::string("buffer-index")
#define VOP_ARG_VSM         std::string("vsm")
#define VOP_ARG_X           std::string("x")
#define VOP_ARG_Y           std::string("y")
#define VOP_ARG_WIDTH       std::string("width")
#define VOP_ARG_HEIGHT      std::string("height")
#define VOP_ARG_WINDOWNAME  std::string("window-name")
#define VOP_ARG_FOV_VERT    std::string("fov-vert")
#define VOP_ARG_FOV_HORIZ   std::string("fov-horiz")

/**
 * All visual operations are performed IN-PLACE. That is, the target image is
 * altered by the operation. When passing in `data_dict` based arguments, 
 * arguments which have default values can be given a NULL value, but ALL 
 * arguments must be passed in.
 */
namespace visual_ops
{
    /**
     * @brief Pull an image from visual sensory memory. The root node always has
     *        this operation.
     * @param args
     *        `int buffer_index`: The index of the VSM buffer which should be
     *         retrieved. Optional, defaults to 0.
     *        `visual_sensory_memory* vsm`: Pointer to vsm
     *        `opencv_image* target` New `opencv_image` to copy the VSM image into
     */ 
    void get_from_vsm(data_dict args);
    
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
     * @brief Display the target image, opening a new window if needed.
     * @param args
     *        `std::string windowName`: The name to give the display window
     *        `opencv_image* target`: The image to display
     */
    void display_image(data_dict args);

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
    void gaussian_blur(data_dict args);

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
     * @brief Crop the target image to the specified rectangle.
     * @param args
     *        `int x`: x-coord of the top-left corner of the rectangle
     *        `int y`: y-coord of the top-left corner of the rectangle
     *        `int width`: width of the rectangle
     *        `int height`: height of the rectangle
     *        `opencv_image* target`: The target imager to crop
     */
    void crop_to_ROI(data_dict args);

    /**
     * @brief Finds the global minimum and maximum in an array. 
     * @param args Map of non-image arguments to method:
     *        `double* minval`: pointer to the returned minimum value
     *        `double* maxval`: pointer to the returned maximum value
     *        `int* minloc_x`: pointer to the returned minimum location's x coordinate
     *        `int* minloc_y`: pointer to the returned minimum location's y coordinate
     *        `int* maxloc_x`: pointer to the returned maximum location's x coordinate
     *        `int* maxloc_y`: pointer to the returned maximum location's y coordinate
     *        `opencv_image* target`: The single-channel input image to find
     *                                min/max of
     */
    void min_max_loc(data_dict args);

        ////////////////////////////////////////////////////////////
    // DEFINE METADATA AND LOOKUP TABLE FOR VISUAL OPERATIONS //
    ////////////////////////////////////////////////////////////

    // Define argument types enum
    enum ArgType { INT_ARG, DOUBLE_ARG, STRING_ARG,  // standard types
                   IMAGE_ARG,       // an integer which represents the ID of another
                                    // vop node. -1 indicates no parent
                   VSM_ARG,         // pointer to VSM
                   VWM_ARG,         // pointer to VWM
                   VLTM_ARG,        // pointer to VLTM
                   WM_ARG,          // pointer to WM
                };

    // Define argument directionalities enum
    enum ArgDirection { INPUT_ARG, OUTPUT_ARG, INOUT_ARG};

    // Define argument optionality enum
    enum ArgOptionality { REQUIRED_ARG, OPTIONAL_ARG };
    
    // Define a struct to hold the parameter metadata for every parameter of a
    // VOp  (i.e., the name, type, directionality, and optionality of the parameters)
    struct vop_params_metadata {
        void (*vop_function)(data_dict);
        int num_params;
        std::vector<std::string> param_names;
        std::vector<ArgType> param_types;
        std::vector<ArgDirection> param_direction;
        std::vector<ArgOptionality> param_optionalities;
    };

    // VISUAL OPERATIONS METADATA DEFINITIONS
    /////////////////////////////////////////

    // GET FROM VSM
    inline vop_params_metadata get_from_vsm_metadata = {
        /* vop_function = */        get_from_vsm,
        /* num_params = */          3,
        /* param_names = */         {VOP_ARG_BUFFERINDEX, VOP_ARG_VSM, VOP_ARG_TARGET},
        /* param_types = */         {INT_ARG, VSM_ARG, IMAGE_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {OPTIONAL_ARG, REQUIRED_ARG, OPTIONAL_ARG}
    };

    // LOAD FROM FILE
    inline vop_params_metadata load_from_file_metadata = {
        /* vop_function = */        load_from_file,
        /* num_params = */          2,
        /* param_names = */         {VOP_ARG_FILEPATH, VOP_ARG_TARGET},
        /* param_types = */         {STRING_ARG, IMAGE_ARG},
        /* param_directions */      {INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, OPTIONAL_ARG}
    };

    // SAVE TO FILE
    inline vop_params_metadata save_to_file_metadata = {
        /* vop_function = */        save_to_file,
        /* num_params = */          2,
        /* param_names = */         {VOP_ARG_FILEPATH, VOP_ARG_TARGET},
        /* param_types = */         {STRING_ARG, IMAGE_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG}
    };

    // DISPLAY IMAGE
    inline vop_params_metadata display_image_metadata = {
        /* vop_function = */        display_image,
        /* num_params = */          2,
        /* param_names = */         {VOP_ARG_WINDOWNAME, VOP_ARG_TARGET},
        /* param_types = */         {STRING_ARG, IMAGE_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG}
    };

    // IDENTITY
    inline vop_params_metadata identity_metadata = {
        /* vop_function = */        identity,
        /* num_params = */          1,
        /* param_names = */         {VOP_ARG_TARGET},
        /* param_types = */         {IMAGE_ARG},
        /* param_directions */      {INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG}
    };

    // BLUR
    inline vop_params_metadata blur_metadata = {
        /* vop_function = */        blur,
        /* num_params = */          6,
        /* param_names = */         {VOP_ARG_SIZEX, VOP_ARG_SIZEY, VOP_ARG_ANCHORX, VOP_ARG_ANCHORY, VOP_ARG_BORDERTYPE, VOP_ARG_TARGET},
        /* param_types = */         {INT_ARG, INT_ARG, INT_ARG, INT_ARG, INT_ARG, IMAGE_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // GAUSSIAN BLUR
    inline vop_params_metadata gaussian_blur_metadata = {
        /* vop_function = */        gaussian_blur,
        /* num_params = */          6,
        /* param_names = */         {VOP_ARG_SIZEX, VOP_ARG_SIZEY, VOP_ARG_SIGMAX, VOP_ARG_SIGMAY, VOP_ARG_BORDERTYPE, VOP_ARG_TARGET},
        /* param_types = */         {INT_ARG, INT_ARG, DOUBLE_ARG, DOUBLE_ARG, INT_ARG, IMAGE_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // GREYSCALE
    inline vop_params_metadata greyscale_metadata = {
        /* vop_function = */        greyscale,
        /* num_params = */          1,
        /* param_names = */         {VOP_ARG_TARGET},
        /* param_types = */         {IMAGE_ARG},
        /* param_directions */      {INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG}
    };

    // THRESHOLD
    inline vop_params_metadata threshold_metadata = {
        /* vop_function = */        threshold,
        /* num_params = */          4,
        /* param_names = */         {VOP_ARG_THRESH, VOP_ARG_MAXVAL, VOP_ARG_TYPE, VOP_ARG_TARGET},
        /* param_types = */         {DOUBLE_ARG, DOUBLE_ARG, INT_ARG, IMAGE_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // MATCH TEMPLATE
    inline vop_params_metadata match_template_metadata = {
        /* vop_function = */        match_template,
        /* num_params = */          3,
        /* param_names = */         {VOP_ARG_METHOD, VOP_ARG_TARGET, VOP_ARG_TEMPLATE},
        /* param_types = */         {INT_ARG, IMAGE_ARG, IMAGE_ARG},
        /* param_directions */      {INPUT_ARG, INOUT_ARG, INPUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // CROP TO ROI
    inline vop_params_metadata crop_to_roi_metadata = {
        /* vop_function = */        crop_to_ROI,
        /* num_params = */          5,
        /* param_names = */         {VOP_ARG_X, VOP_ARG_Y, VOP_ARG_WIDTH, VOP_ARG_HEIGHT, VOP_ARG_TARGET},
        /* param_types = */         {INT_ARG, INT_ARG, INT_ARG, INT_ARG, IMAGE_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // MIN MAX LOC
    inline vop_params_metadata min_max_loc_metadata = {
        /* vop_function = */        min_max_loc,
        /* num_params = */          7,
        /* param_names = */         {VOP_ARG_MINVAL, VOP_ARG_MAXVAL, VOP_ARG_MINLOCX, VOP_ARG_MINLOCY, VOP_ARG_MAXLOCX, VOP_ARG_MAXLOCY, VOP_ARG_TARGET},
        /* param_types = */         {DOUBLE_ARG,     DOUBLE_ARG,     INT_ARG,         INT_ARG,         INT_ARG,         INT_ARG,         IMAGE_ARG},
        /* param_directions */      {OUTPUT_ARG,     OUTPUT_ARG,     OUTPUT_ARG,      OUTPUT_ARG,      OUTPUT_ARG,      OUTPUT_ARG,      INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG,   REQUIRED_ARG,   REQUIRED_ARG,    REQUIRED_ARG,    REQUIRED_ARG,    REQUIRED_ARG,    REQUIRED_ARG}
    };

    // CREATE LOOKUP TABLE MAPPING OPERATION NAMES TO METADATA
    //////////////////////////////////////////////////////////
    inline std::unordered_map<std::string, vop_params_metadata> vops_param_table({
        {VOP_GET_FROM_VSM, get_from_vsm_metadata},
        {VOP_LOAD_FROM_FILE, load_from_file_metadata},
        {VOP_SAVE_TO_FILE, save_to_file_metadata},
        {VOP_DISPLAY_IMAGE, display_image_metadata},
        {VOP_IDENTITY, identity_metadata},
        {VOP_BLUR, blur_metadata},
        {VOP_GAUSSIANBLUR, gaussian_blur_metadata},
        {VOP_GREYSCALE, greyscale_metadata},
        {VOP_THRESHOLD, threshold_metadata},
        {VOP_MATCH_TEMPLATE, match_template_metadata},
        {VOP_CROP_TO_ROI, crop_to_roi_metadata},
        {VOP_MIN_MAX_LOC, min_max_loc_metadata}
    });

} // namespace visual_ops

#endif