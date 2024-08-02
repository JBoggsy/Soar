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
#define VOP_GET_FROM_VIB            std::string("get-from-vib")
#define VOP_LOAD_FROM_FILE          std::string("load-from-file")
#define VOP_SAVE_TO_FILE            std::string("save-to-file")
#define VOP_BLUR                    std::string("blur")
#define VOP_DISPLAY_IMAGE           std::string("display-image")
#define VOP_IDENTITY                std::string("identity")
#define VOP_GAUSSIANBLUR            std::string("gaussian-blur")
#define VOP_GREYSCALE               std::string("greyscale")
#define VOP_THRESHOLD               std::string("threshold")
#define VOP_FLIP_IMAGE              std::string("flip-image")
#define VOP_CREATE_INT_FILLED_MAT   std::string("create-int-filled-mat")
#define VOP_CREATE_FLOAT_FILLED_MAT std::string("create-float-filled-mat")
#define VOP_CREATE_X_COORD_MAT      std::string("create-x-coord-mat")
#define VOP_CREATE_Y_COORD_MAT      std::string("create-y-coord-mat")
#define VOP_EXTRACT_CHANNEL         std::string("extract-channel")
#define VOP_EXTRACT_CHANNELS        std::string("extract-channels")
#define VOP_STACK_MATRICES          std::string("stack-matrices")
#define VOP_ADD_MATS                std::string("add-mats")
#define VOP_SUB_MATS                std::string("sub-mats")
#define VOP_MUL_MATS                std::string("mul-mats")
#define VOP_DIV_MATS                std::string("div-mats")
#define VOP_APPLY_UNARY_OP          std::string("apply-unary-op")
#define VOP_MATCH_TEMPLATE          std::string("match-template")
#define VOP_CROP_TO_ROI             std::string("crop-to-roi")
#define VOP_MIN_MAX_LOC             std::string("min-max-loc")

// Define argument names as strings
#define VOP_ARG_A           std::string("a")
#define VOP_ARG_ANCHORX     std::string("anchor-x")
#define VOP_ARG_ANCHORY     std::string("anchor-y")
#define VOP_ARG_AXES        std::string{"axes"}
#define VOP_ARG_B           std::string("b")
#define VOP_ARG_BORDERTYPE  std::string("border-type")
#define VOP_ARG_BUFFERINDEX std::string("buffer-index")
#define VOP_ARG_CHANNEL     std::string("channel")
#define VOP_ARG_END         std::string("end")
#define VOP_ARG_FILEPATH    std::string("filepath")
#define VOP_ARG_FILL_VAL    std::string("fill-val")
#define VOP_ARG_FOV_VERT    std::string("fov-vert")
#define VOP_ARG_FOV_HORIZ   std::string("fov-horiz")
#define VOP_ARG_HEIGHT      std::string("height")
#define VOP_ARG_MAXLOCX     std::string("maxloc-x")
#define VOP_ARG_MAXLOCY     std::string("maxloc-y")
#define VOP_ARG_MAXVAL      std::string("maxval")
#define VOP_ARG_METHOD      std::string("method")
#define VOP_ARG_MINLOCX     std::string("minloc-x")
#define VOP_ARG_MINLOCY     std::string("minloc-y")
#define VOP_ARG_MINVAL      std::string("minval")
#define VOP_ARG_OP          std::string("unary-op")
#define VOP_ARG_SIGMAX      std::string("sigma-x")
#define VOP_ARG_SIGMAY      std::string("sigma-y")
#define VOP_ARG_SIZEX       std::string("size-x")
#define VOP_ARG_SIZEY       std::string("size-y")
#define VOP_ARG_START       std::string("start")
#define VOP_ARG_TEMPLATE    std::string("template")
#define VOP_ARG_SOURCE      std::string("source")
#define VOP_ARG_THRESH      std::string("thresh")
#define VOP_ARG_TYPE        std::string("type")
#define VOP_ARG_VIBID       std::string("vib-id")
#define VOP_ARG_VIBMGR      std::string("vib-manager")
#define VOP_ARG_WIDTH       std::string("width")
#define VOP_ARG_WINDOWNAME  std::string("window-name")
#define VOP_ARG_X           std::string("x")
#define VOP_ARG_Y           std::string("y")

/**
 * All visual operations are performed IN-PLACE. That is, the source image is
 * altered by the operation. When passing in `data_dict` based arguments,
 * arguments which have default values can be given a NULL value, but ALL
 * arguments must be passed in.
 */
namespace visual_ops
{
    ///////////////////////////
    // VISUAL INPUTS/OUTPUTS //
    ///////////////////////////

    /**
     * @brief Pull an image from a visual input buffer. The root node always has
     *        this operation.
     * @param args
     *        `std::string vib-id`: The id of the VIB to retrieve from.
     *        `int buffer-index`: The index of the frame which should be retrieved.
     *        Optional, defaults to 0.
     *        `visual_input_buffer_manager* vib-manager`: Pointer to the `visual_input_buffer_manager`
     *        `opencv_image* source` New `opencv_image` to copy the VIB image into
     */
    void get_from_vib(data_dict args);

    /**
     * @brief Load an image from a file
     * @param args
     *        `std::string filepath`: The absolute path to the source file
     *        `opencv_image* source` Object to load the specified file into
     */
    void load_from_file(data_dict args);

    /**
     * @brief Save an image to a file
     * @param args
     *        `std::string filepath`: The absolute path to save location
     *        `opencv_image* source` Object to save into the specified file
     */
    void save_to_file(data_dict args);

    /**
     * @brief Display the source image, opening a new window if needed.
     * @param args
     *        `std::string windowName`: The name to give the display window
     *        `opencv_image* source`: The image to display
     */
    void display_image(data_dict args);


    ////////////////////////////
    // VISUAL TRANSFORMATIONS //
    ////////////////////////////

    /**
     * @brief Returns the input image without changes, used to add a no-op node.
     * @param args
     *        `opencv_image* source`: Image to do nothing with
     */
    void identity(data_dict args);

    /**
     * @brief Blur the single source image
     * @param args
     *        `int size-x`: Blurring kernel size along the x dimension
     *        `int size-y`: Blurring kernel size along the y dimension
     *        `int anchor-x`: Anchor point x coordinate; default value of-1 means that the anchor is at the kernel's center
     *        `int anchor-y`: Anchor point y coordinate; default value of-1 means that the anchor is at the kernel's center
     *        `int borderType`: Border mode used to extrapolate pixels outside of the image
     *        `opencv_image* source`: The image to blur
     */
    void blur(data_dict args);

    /**
     * @brief Blur the single source image using a Gaussian filter.
     *
     * @note ksize-x and ksize-yt can differ but they both must be positive and odd. Alternatively, they can be zeroes and
     *       then they are computed from sigma.
     * @param args
     *        `int ksize-x`: Gaussian kernel width
     *        `int ksize-y`: Gaussian kernel height
     *        `double sigmaX`: Gaussian kernel standard deviation in X direction.
     *        `double sigmaY`: Gaussian kernel standard deviation in Y direction; if sigmaY is zero, it is set to be equal to sigmaX, if both sigmas are zeros, they are computed from ksize.width and ksize.height, respectively (see getGaussianKernel for details); to fully control the result regardless of possible future modifications of all this semantics, it is recommended to specify all of ksize, sigmaX, and sigmaY.
     *        `int borderType`: Border mode used to extrapolate pixels outside of the image
     *        `opencv_image* source`: The image to blur
     */
    void gaussian_blur(data_dict args);

    /**
     * @brief Converts the source image into greyscale.
     * @param args
     *        `opencv_image* source`: The image to convert to greyscale
     */
    void greyscale(data_dict args);

    /**
     * @brief Binarize a grayscale image according to a threshold
     * @param args
     *        `double thresh`: The threshold value.
     *        `double maxval`: maximum value to use with the THRESH_BINARY and THRESH_BINARY_INV thresholding types.
     *        `int type`: The thresholding type.
     *        `opencv_image* source`: The image to greyscale
     */
    void threshold(data_dict args);

    /**
     * @brief Flip the image across the x or y axis, or both.
     *
     * @param args
     *      `std:string axes`: One of `x`, `y`, `xy,` or `yx`. Indicates axes to flip across.
     *      `opencv_image* source`: The image to flip
     */
    void flip_image(data_dict args);


    /////////////////////
    // MATRIX CREATION //
    /////////////////////

    /**
     * @brief Create a matrix of the given size filled with the given value.
     *
     * @param args
     *      `int size-x`: Width of the new matrix in pixels
     *      `int size-y`: Height of the new matrix in pixels
     *      `int fill-val`: Value to fill the matrix with
     *      `opencv_image* source`: Matrix filled with the given value
     */
    void create_int_filled_mat(data_dict args);

    /**
     * @brief Create a matrix of the given size filled with the given value.
     *
     * @param args
     *      `int size-x`: Width of the new matrix in pixels
     *      `int size-y`: Height of the new matrix in pixels
     *      `float fill-val`: Value to fill the matrix with
     *      `opencv_image* source`: Matrix filled with the given value
     */
    void create_float_filled_mat(data_dict args);

    /**
     * @brief Create a matrix of the given size where the value of a each cell is its x coordinate.
     *
     * @param args
     *      `int size-x`: Width of the new matrix in pixels
     *      `int size-y`: Height of the new matrix in pixels
     *      `opencv_image* source`: Filled matrix
     */
    void create_x_coord_mat(data_dict args);

    /**
     * @brief Create a matrix of the given size where the value of a each cell is its y coordinate.
     *
     * @param args
     *      `int size-x`: Width of the new matrix in pixels
     *      `int size-y`: Height of the new matrix in pixels
     *      `opencv_image* source`: Filled matrix
     */
    void create_y_coord_mat(data_dict args);


    ////////////////////////
    // MATRIX MODIFICTION //
    ////////////////////////

    /**
     * @brief Combine two matrices by stacking them channel-wise.
     *
     * @details This is essentially the cv::merge method, but slightly more limited. It allows the
     * agent to combine two matrices of the same size and depth by "stacking" them. For example, if
     * matrix A has three channels R,G,B, and matrix B has three channels X,Y,Z, then stacking A on
     * B creates a new matrix with six channels R,G,B,X,Y,Z.
     *
     * @param args
     *      `opencv_image* a`: The matrix whose channels will come first in the new matrix
     *      `opencv_image* b`: The matrix whose channels will come last in the new matrix
     *      `opencv_image* source`: The matrix resulting from the stacking of `a` on `b`.
     */
    void stack_matrices(data_dict args);

    /**
     * @brief Extract a single channel from the source image into a new matrix.
     *
     * @param args
     *      `int channel`: THe (0-based) index of the channel to extract
     *      `opencv_image* source`: The image to extract channels from
     */
    void extract_channel(data_dict args);

    /**
     * @brief Extract one or more sequential channels from the source image into a new matrix.
     *
     * @param args
     *      `int start`: the index of the lowest channel to extract, the "bottom" of the extracted channels
     *      `int end`: the index of the highest channel to extract, the "bottom" of the extracted channels
     *      `opencv_image* source`: The image to extract channels from
     */
    void extract_channels(data_dict args);


    /////////////////////////////
    // MATHEMATICAL PRIMITIVES //
    /////////////////////////////

    /**
     * @brief Add two matrices together element-wise as `c[x,y] = a[x,y]+b[x,y]`.
     *
     * @param args
     *      `opencv_image* a`: First addend
     *      `opencv_image* b`: Second addend
     *      `opencv_image* source`: Result of element-wise addition
     */
    void add_mats(data_dict args);

    /**
     * @brief Subtract matrix `a` from matrix `b` element-wise as `c[x,y] = a[x,y]-b[x,y]`.
     *
     * @param args
     *      `opencv_image* a`: Multiplicand
     *      `opencv_image* b`: Multiplier
     *      `opencv_image* source`: Result of element-wise subtraction
     */
    void sub_mats(data_dict args);

    /**
     * @brief Multiply matrix `a` by matrix `b` element-wise as `c[x,y] = a[x,y]*b[x,y]`.
     *
     * @param args
     *      `opencv_image* a`: Minuend
     *      `opencv_image* b`: Subtrahend
     *      `opencv_image* source`: Result of element-wise multiplication
     */
    void mul_mats(data_dict args);

    /**
     * @brief Divide matrix `a` by matrix `b` element-wise as `c[x,y] = a[x,y]/b[x,y]`.
     *
     * @param args
     *      `opencv_image* a`: Dividend
     *      `opencv_image* b`: Divisor
     *      `opencv_image* source`: Result of element-wise division
     */
    void div_mats(data_dict args);

    /**
     * @brief Apply the specifid unary operation elementwise across the source.
     *
     * @note Presently only applicable to single-channel arrays.
     *
     * @param args
     *      `opencv_image* source`: The matrix to apply the operation to
     *      `std::string op`: The name of the operation to apply. One of:
     *          "negate"
     *          "cos"
     *          "sin"
     */
    void apply_unary_op(data_dict args);


    //////////////////////
    // OBJECT DETECTION //
    //////////////////////

    /**
     * @brief Match a template image and return a new, single-channel image of
     *        comparison results. The returned image will be a single-channel
     *        32-bit floating-point image. If the source image is W×H and template
     *        is w×h , then result is (W−w+1)×(H−h+1).
     * @param args
     *        `int method`: Parameter specifying the comparison method
     *        `opencv_image* source`: The image to be searched for template matches.
     *        `opencv_image* template`: The template image to search for.
     */
    void match_template(data_dict args);

    /**
     * @brief Crop the source image to the specified rectangle.
     * @param args
     *        `int x`: x-coord of the top-left corner of the rectangle
     *        `int y`: y-coord of the top-left corner of the rectangle
     *        `int width`: width of the rectangle
     *        `int height`: height of the rectangle
     *        `opencv_image* source`: The source imager to crop
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
     *        `opencv_image* source`: The single-channel input image to find
     *                                min/max of
     */
    void min_max_loc(data_dict args);

    ////////////////////////////////////////////////////////////
    // DEFINE METADATA AND LOOKUP TABLE FOR VISUAL OPERATIONS //
    ////////////////////////////////////////////////////////////

    // Define argument types enum
    enum ArgType {
        // SIMPLE DATA TYPES
        INT_ARG,
        DOUBLE_ARG,
        STRING_ARG,
        // IMAGE TYPES
        NODE_ID_ARG, // an integer which represents the ID of another
                     // vop node. -1 indicates no parent
        // MEMORY POINTERS
        VIBMGR_ARG,      // pointer to the VIB manager
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

    // GET FROM VIB
    inline vop_params_metadata get_from_vib_metadata = {
        /* vop_function = */        get_from_vib,
        /* num_params = */          4,
        /* param_names = */         {VOP_ARG_VIBID, VOP_ARG_BUFFERINDEX, VOP_ARG_VIBMGR, VOP_ARG_SOURCE},
        /* param_types = */         {STRING_ARG, INT_ARG, VIBMGR_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, OPTIONAL_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // LOAD FROM FILE
    inline vop_params_metadata load_from_file_metadata = {
        /* vop_function = */        load_from_file,
        /* num_params = */          2,
        /* param_names = */         {VOP_ARG_FILEPATH, VOP_ARG_SOURCE},
        /* param_types = */         {STRING_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, OPTIONAL_ARG}
    };

    // SAVE TO FILE
    inline vop_params_metadata save_to_file_metadata = {
        /* vop_function = */        save_to_file,
        /* num_params = */          2,
        /* param_names = */         {VOP_ARG_FILEPATH, VOP_ARG_SOURCE},
        /* param_types = */         {STRING_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG}
    };

    // DISPLAY IMAGE
    inline vop_params_metadata display_image_metadata = {
        /* vop_function = */        display_image,
        /* num_params = */          2,
        /* param_names = */         {VOP_ARG_WINDOWNAME, VOP_ARG_SOURCE},
        /* param_types = */         {STRING_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG}
    };

    // IDENTITY
    inline vop_params_metadata identity_metadata = {
        /* vop_function = */        identity,
        /* num_params = */          1,
        /* param_names = */         {VOP_ARG_SOURCE},
        /* param_types = */         {NODE_ID_ARG},
        /* param_directions */      {INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG}
    };

    // BLUR
    inline vop_params_metadata blur_metadata = {
        /* vop_function = */        blur,
        /* num_params = */          6,
        /* param_names = */         {VOP_ARG_SIZEX, VOP_ARG_SIZEY, VOP_ARG_ANCHORX, VOP_ARG_ANCHORY, VOP_ARG_BORDERTYPE, VOP_ARG_SOURCE},
        /* param_types = */         {INT_ARG, INT_ARG, INT_ARG, INT_ARG, INT_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // GAUSSIAN BLUR
    inline vop_params_metadata gaussian_blur_metadata = {
        /* vop_function = */        gaussian_blur,
        /* num_params = */          6,
        /* param_names = */         {VOP_ARG_SIZEX, VOP_ARG_SIZEY, VOP_ARG_SIGMAX, VOP_ARG_SIGMAY, VOP_ARG_BORDERTYPE, VOP_ARG_SOURCE},
        /* param_types = */         {INT_ARG, INT_ARG, DOUBLE_ARG, DOUBLE_ARG, INT_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // GREYSCALE
    inline vop_params_metadata greyscale_metadata = {
        /* vop_function = */        greyscale,
        /* num_params = */          1,
        /* param_names = */         {VOP_ARG_SOURCE},
        /* param_types = */         {NODE_ID_ARG},
        /* param_directions */      {INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG}
    };

    // THRESHOLD
    inline vop_params_metadata threshold_metadata = {
        /* vop_function = */        threshold,
        /* num_params = */          4,
        /* param_names = */         {VOP_ARG_THRESH, VOP_ARG_MAXVAL, VOP_ARG_TYPE, VOP_ARG_SOURCE},
        /* param_types = */         {DOUBLE_ARG, DOUBLE_ARG, INT_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // FLIP IMAGE
    inline vop_params_metadata flip_image_metadata = {
        /* vop_function = */        flip_image,
        /* num_params = */          2,
        /* param_names = */         {VOP_ARG_AXES, VOP_ARG_SOURCE},
        /* param_types = */         {STRING_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG}
    };

    // CREATE INT FILLED MATRIX
    inline vop_params_metadata create_int_filled_mat_metadata = {
        /* vop_function = */        create_int_filled_mat,
        /* num_params = */          4,
        /* param_names = */         {VOP_ARG_SIZEX, VOP_ARG_SIZEY, VOP_ARG_FILL_VAL, VOP_ARG_SOURCE},
        /* param_types = */         {INT_ARG, INT_ARG, INT_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // CREATE FLOAT FILLED MATRIX
    inline vop_params_metadata create_float_filled_mat_metadata = {
        /* vop_function = */        create_float_filled_mat,
        /* num_params = */          4,
        /* param_names = */         {VOP_ARG_SIZEX, VOP_ARG_SIZEY, VOP_ARG_FILL_VAL, VOP_ARG_SOURCE},
        /* param_types = */         {INT_ARG, INT_ARG, DOUBLE_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // CREATE X COORDINATE FILLED MATRIX
    inline vop_params_metadata create_x_coord_mat_metadata = {
        /* vop_function = */        create_x_coord_mat,
        /* num_params = */          3,
        /* param_names = */         {VOP_ARG_SIZEX, VOP_ARG_SIZEY, VOP_ARG_SOURCE},
        /* param_types = */         {INT_ARG, INT_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // CREATE Y COORDINATE FILLED MATRIX
    inline vop_params_metadata create_y_coord_mat_metadata = {
        /* vop_function = */        create_y_coord_mat,
        /* num_params = */          3,
        /* param_names = */         {VOP_ARG_SIZEX, VOP_ARG_SIZEY, VOP_ARG_SOURCE},
        /* param_types = */         {INT_ARG, INT_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // STACK MATRICES
    inline vop_params_metadata stack_matrices_metadata = {
        /* vop_function = */        stack_matrices,
        /* num_params = */          3,
        /* param_names = */         {VOP_ARG_A, VOP_ARG_B, VOP_ARG_SOURCE},
        /* param_types = */         {NODE_ID_ARG, NODE_ID_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // EXTRACT CHANNEL FROM MATRIX
    inline vop_params_metadata extract_channel_metadata = {
        /* vop_function = */        extract_channel,
        /* num_params = */          2,
        /* param_names = */         {VOP_ARG_CHANNEL, VOP_ARG_SOURCE},
        /* param_types = */         {INT_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG}
    };

    // EXTRACT CHANNELS FROM MATRIX
    inline vop_params_metadata extract_channels_metadata = {
        /* vop_function = */        extract_channels,
        /* num_params = */          3,
        /* param_names = */         {VOP_ARG_START, VOP_ARG_END, VOP_ARG_SOURCE},
        /* param_types = */         {INT_ARG, INT_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // ADD MATRICES
    inline vop_params_metadata add_mats_metadata = {
        /* vop_function = */        add_mats,
        /* num_params = */          3,
        /* param_names = */         {VOP_ARG_A, VOP_ARG_B, VOP_ARG_SOURCE},
        /* param_types = */         {NODE_ID_ARG, NODE_ID_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // SUBTRACT MATRICES
    inline vop_params_metadata sub_mats_metadata = {
        /* vop_function = */        sub_mats,
        /* num_params = */          3,
        /* param_names = */         {VOP_ARG_A, VOP_ARG_B, VOP_ARG_SOURCE},
        /* param_types = */         {NODE_ID_ARG, NODE_ID_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // MULTIPLY MATRICES
    inline vop_params_metadata mul_mats_metadata = {
        /* vop_function = */        mul_mats,
        /* num_params = */          3,
        /* param_names = */         {VOP_ARG_A, VOP_ARG_B, VOP_ARG_SOURCE},
        /* param_types = */         {NODE_ID_ARG, NODE_ID_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // DIVIDE MATRICES
    inline vop_params_metadata div_mats_metadata = {
        /* vop_function = */        div_mats,
        /* num_params = */          3,
        /* param_names = */         {VOP_ARG_A, VOP_ARG_B, VOP_ARG_SOURCE},
        /* param_types = */         {NODE_ID_ARG, NODE_ID_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // APPLY UNARY OPERATION
    inline vop_params_metadata apply_unary_op_metadata = {
        /* vop_function = */        apply_unary_op,
        /* num_params = */          2,
        /* param_names = */         {VOP_ARG_OP, VOP_ARG_SOURCE},
        /* param_types = */         {STRING_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG}
    };

    // MATCH TEMPLATE
    inline vop_params_metadata match_template_metadata = {
        /* vop_function = */        match_template,
        /* num_params = */          3,
        /* param_names = */         {VOP_ARG_METHOD, VOP_ARG_SOURCE, VOP_ARG_TEMPLATE},
        /* param_types = */         {INT_ARG, NODE_ID_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INOUT_ARG, INPUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // CROP TO ROI
    inline vop_params_metadata crop_to_roi_metadata = {
        /* vop_function = */        crop_to_ROI,
        /* num_params = */          5,
        /* param_names = */         {VOP_ARG_X, VOP_ARG_Y, VOP_ARG_WIDTH, VOP_ARG_HEIGHT, VOP_ARG_SOURCE},
        /* param_types = */         {INT_ARG, INT_ARG, INT_ARG, INT_ARG, NODE_ID_ARG},
        /* param_directions */      {INPUT_ARG, INPUT_ARG, INPUT_ARG, INPUT_ARG, INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG, REQUIRED_ARG}
    };

    // MIN MAX LOC
    inline vop_params_metadata min_max_loc_metadata = {
        /* vop_function = */        min_max_loc,
        /* num_params = */          7,
        /* param_names = */         {VOP_ARG_MINVAL, VOP_ARG_MAXVAL, VOP_ARG_MINLOCX, VOP_ARG_MINLOCY, VOP_ARG_MAXLOCX, VOP_ARG_MAXLOCY, VOP_ARG_SOURCE},
        /* param_types = */         {DOUBLE_ARG,     DOUBLE_ARG,     INT_ARG,         INT_ARG,         INT_ARG,         INT_ARG,         NODE_ID_ARG},
        /* param_directions */      {OUTPUT_ARG,     OUTPUT_ARG,     OUTPUT_ARG,      OUTPUT_ARG,      OUTPUT_ARG,      OUTPUT_ARG,      INOUT_ARG},
        /* param_optionalities = */ {REQUIRED_ARG,   REQUIRED_ARG,   REQUIRED_ARG,    REQUIRED_ARG,    REQUIRED_ARG,    REQUIRED_ARG,    REQUIRED_ARG}
    };

    // CREATE LOOKUP TABLE MAPPING OPERATION NAMES TO METADATA
    //////////////////////////////////////////////////////////
    inline std::unordered_map<std::string, vop_params_metadata> vops_param_table({
        {VOP_GET_FROM_VIB, get_from_vib_metadata},
        {VOP_LOAD_FROM_FILE, load_from_file_metadata},
        {VOP_SAVE_TO_FILE, save_to_file_metadata},
        {VOP_DISPLAY_IMAGE, display_image_metadata},
        {VOP_IDENTITY, identity_metadata},
        {VOP_BLUR, blur_metadata},
        {VOP_GAUSSIANBLUR, gaussian_blur_metadata},
        {VOP_GREYSCALE, greyscale_metadata},
        {VOP_THRESHOLD, threshold_metadata},
        {VOP_FLIP_IMAGE, flip_image_metadata},
        {VOP_CREATE_INT_FILLED_MAT, create_int_filled_mat_metadata},
        {VOP_CREATE_FLOAT_FILLED_MAT, create_float_filled_mat_metadata},
        {VOP_CREATE_X_COORD_MAT, create_x_coord_mat_metadata},
        {VOP_CREATE_Y_COORD_MAT, create_y_coord_mat_metadata},
        {VOP_STACK_MATRICES, stack_matrices_metadata},
        {VOP_EXTRACT_CHANNEL, extract_channel_metadata},
        {VOP_EXTRACT_CHANNELS, extract_channels_metadata},
        {VOP_ADD_MATS, add_mats_metadata},
        {VOP_SUB_MATS, sub_mats_metadata},
        {VOP_MUL_MATS, mul_mats_metadata},
        {VOP_DIV_MATS, div_mats_metadata},
        {VOP_APPLY_UNARY_OP, apply_unary_op_metadata},
        {VOP_MATCH_TEMPLATE, match_template_metadata},
        {VOP_CROP_TO_ROI, crop_to_roi_metadata},
        {VOP_MIN_MAX_LOC, min_max_loc_metadata}
    });

} // namespace visual_ops

#endif
