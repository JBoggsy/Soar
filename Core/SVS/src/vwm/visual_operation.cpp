#include <iostream>
#include <math.h>
#include <queue>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "visual_operation.h"
#include "visual_input_buffer.h"
#include "visual_long_term_memory.h"
#include "exact_visual_concept_descriptor.h"
#include "vae_visual_concept_descriptor.h"
#include "image.h"
#include "latent_representation.h"
#include "token_sequence.h"
#include "object_representation.h"


namespace visual_ops
{
    ///////////////////////////
    // SECTION IMAGE SOURCES //
    ///////////////////////////

    void get_from_vib(data_dict args) {
        std::string vib_id;
        int buffer_index;
        visual_input_buffer_manager* vibmgr;
        opencv_image* image;

        vib_id = *(std::string*)(args[VOP_ARG_VIBID]);

        if (args[VOP_ARG_BUFFERINDEX] == NULL) {
            buffer_index = 0;
        } else {
            buffer_index = *(int*)args[VOP_ARG_BUFFERINDEX];
        }

        vibmgr = (visual_input_buffer_manager*)args[VOP_ARG_VIBMGR];

        image = (opencv_image*)args[VOP_ARG_SOURCE];

        cv::Mat converted_image;
        vibmgr->get_visual_input_buffer(vib_id)->get_frame(buffer_index)->get_image()->convertTo(converted_image, CV_32F);

        image->set_image(&converted_image);
        cv::Size image_shape = image->get_image()->size();
        // printf("VSM image shape: (%d, %d, %d), type: %d\n", image_shape.width, image_shape.height, image->get_image()->channels(), image->get_image()->type());
    }

    void load_from_file(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        std::string* filepath = (std::string*)args[VOP_ARG_FILEPATH];

        image->update_image(cv::imread(*filepath, cv::IMREAD_UNCHANGED));
        cv::Size image_shape = image->get_image()->size();
    }

    void save_to_file(data_dict args) {
        cv::Mat image = *(((opencv_image*)args[VOP_ARG_SOURCE])->get_image());
        std::string filepath = *(std::string*)args[VOP_ARG_FILEPATH];

        try {
            cv::imwrite(filepath, image);
        } catch (const cv::Exception& ex) {
            printf("Exception converting image to file: %s\n", ex.what());
        }

    }

    void display_image(data_dict args) {
        cv::Mat image = *(((opencv_image*)args[VOP_ARG_SOURCE])->get_image());
        std::string window_name = *(std::string*)args[VOP_ARG_WINDOWNAME];

        cv::imshow(window_name, image);
        cv::waitKey(1);
    }
    //!SECTION VISUAL INPUTS/OUTPUTS


    ////////////////////////////////////
    // SECTION VISUAL TRANSFORMATIONS //
    ////////////////////////////////////

    void identity(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];

        cv::Mat result;
        result = *(image->get_image());
        image->set_image(&result);
    }

    void blur(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        int size_x = *(int*)args[VOP_ARG_SIZEX];
        int size_y = *(int*)args[VOP_ARG_SIZEY];
        cv::Size ksize(size_x, size_y);

        cv::Point anchor;
        if (args[VOP_ARG_ANCHORX] = NULL) {
            anchor.x = *(int*)args[VOP_ARG_ANCHORX];
        } else {
            anchor.x = -1;
        }

        if (args[VOP_ARG_ANCHORY] != NULL) {
            anchor.y = *(int*)args[VOP_ARG_ANCHORY];
        } else {
            anchor.y = -1;
        }

        int borderType;
        if (args[VOP_ARG_BORDERTYPE] != NULL) {
            borderType = *(int*)args[VOP_ARG_BORDERTYPE];
        } else {
            borderType = cv::BORDER_DEFAULT;
        }

        cv::Mat result;
        cv::blur(*image->get_image(), result, ksize, anchor, borderType);
        image->set_image(&result);
    }

    void gaussian_blur(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];

        int size_x = *(int*)args[VOP_ARG_SIZEX];
        int size_y = *(int*)args[VOP_ARG_SIZEY];
        cv::Size ksize(size_x, size_y);

        double sigmaX = *(double*)args[VOP_ARG_SIGMAX];
        double sigmaY = *(double*)args[VOP_ARG_SIGMAY];

        int borderType;
        if (args[VOP_ARG_BORDERTYPE] != NULL) {
            borderType = *(int*)args[VOP_ARG_BORDERTYPE];
        } else {
            borderType = cv::BORDER_DEFAULT;
        }

        cv::Mat result;
        cv::GaussianBlur(*image->get_image(), result, ksize, sigmaX, sigmaY, borderType);
        image->set_image(&result);
    }

    void greyscale(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];

        cv::Mat result;
        cv::cvtColor(*image->get_image(), result, cv::COLOR_BGRA2GRAY);
        image->set_image(&result);
    }

    void threshold(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        double thresh = *(double*)args[VOP_ARG_THRESH];
        double maxval = *(double*)args[VOP_ARG_MAXVAL];
        int type = *(int*)args[VOP_ARG_TYPE];

        cv::Mat result;
        cv::threshold(*image->get_image(), result, thresh, maxval, type);
        image->set_image(&result);
    }

    void flip_image(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        std::string axes = *(std::string*)args[VOP_ARG_AXES];
        int flip_code;
        if (axes.compare("x") == 0) { flip_code = 0; }
        else if (axes.compare("y") == 0) { flip_code = 1; }
        else if ((axes.compare("xy") == 0) || (axes.compare("yx") == 0)) { flip_code=-1; }
        else { return; }

        cv::Mat result;
        cv::flip(*(image->get_image()), result, flip_code);
        image->set_image(&result);
    }

    void rotate_image(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        double amount = *(double*)args[VOP_ARG_AMOUNT];

        cv::Mat rot_mat, result;
        int height, width;
        float center_x, center_y;
        height = image->get_height();
        width = image->get_width();
        center_y = (float)height / 2.0;
        center_x = (float)width / 2.0;
        rot_mat = cv::getRotationMatrix2D(cv::Point2f(center_x, center_y), amount, 1.0);

        float abs_cos = std::abs(rot_mat.at<double>(0, 0));
        float abs_sin = std::abs(rot_mat.at<double>(0, 1));
        int new_width = (int)(height * abs_sin + width * abs_cos + 2);
        int new_height = (int)(height * abs_cos + width * abs_sin + 2);
        rot_mat.at<double>(0, 2) += (new_width - width) / 2;
        rot_mat.at<double>(1, 2) += (new_height - height) / 2;

        cv::warpAffine(*(image->get_image()), result, rot_mat, cv::Size(new_width, new_height), cv::INTER_NEAREST);
        image->set_image(&result);
    }

    void resize_image(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        int size_x = *(int*)args[VOP_ARG_SIZEX];
        int size_y = *(int*)args[VOP_ARG_SIZEY];

        cv::Mat result;
        cv::resize(*(image->get_image()), result, cv::Size(size_x, size_y), 0, 0, cv::INTER_NEAREST);
        image->set_image(&result);
    }

    void scale_image(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        double scale_x = *(double*)args[VOP_ARG_SCALEX];
        double scale_y = *(double*)args[VOP_ARG_SCALEY];

        cv::Mat result;
        cv::resize(*(image->get_image()), result, cv::Size(), scale_x, scale_y, cv::INTER_NEAREST);
        image->set_image(&result);
    }
    //!SECTION

    //////////////////////////////
    // SECTION  MATRIX CREATION //
    //////////////////////////////

    void create_int_filled_mat(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        int size_x =         *(int*)args[VOP_ARG_SIZEX];
        int size_y =         *(int*)args[VOP_ARG_SIZEY];
        int fill =           *(int*)args[VOP_ARG_FILL_VAL];

        cv::Mat new_mat = cv::Mat(size_y, size_x, CV_16S, fill);
        image->set_image(&new_mat);
    }

    void create_float_filled_mat(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        int size_x =         *(int*)args[VOP_ARG_SIZEX];
        int size_y =         *(int*)args[VOP_ARG_SIZEY];
        double fill =        *(double*)args[VOP_ARG_FILL_VAL];

        cv::Mat new_mat = cv::Mat(size_y, size_x, CV_32F, fill);
        image->set_image(&new_mat);
    }

    void create_x_coord_mat(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        int size_x =         *(int*)args[VOP_ARG_SIZEX];
        int size_y =         *(int*)args[VOP_ARG_SIZEY];

        cv::Mat new_mat = cv::Mat(size_y, size_x, CV_32FC1);
        new_mat.forEach<float>([](float &cell_val, const int position[]) -> void {
            cell_val = (float)position[1];
        });
        image->set_image(&new_mat);
    }
    void create_y_coord_mat(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        int size_x =         *(int*)args[VOP_ARG_SIZEX];
        int size_y =         *(int*)args[VOP_ARG_SIZEY];

        cv::Mat new_mat = cv::Mat(size_y, size_x, CV_32FC1);
        new_mat.forEach<float>([](float &cell_val, const int position[]) -> void {
            cell_val = (float)position[0];
        });
        image->set_image(&new_mat);
    }
    //!SECTION MATRIX CREATION

    //////////////////////////////
    // SECTION MATRIX ACCESSING //
    //////////////////////////////

    void stack_matrices(data_dict args) {
        opencv_image* image     = (opencv_image*)args[VOP_ARG_SOURCE];
        opencv_image* a         = (opencv_image*)args[VOP_ARG_A];
        opencv_image* b         = (opencv_image*)args[VOP_ARG_B];

        // Check to ensure depth compatibility
        int a_depth = a->get_image()->depth();
        int b_depth = b->get_image()->depth();
        if (a_depth != b_depth) {
            printf("Failed to mul mats: type error: %d != %d\n", a_depth, b_depth);
            return;
        }

        // Check to ensure size compatibility
        cv::Size a_size = a->get_image()->size();
        cv::Size b_size = b->get_image()->size();
        if ( (a_size.height != b_size.height) || (a_size.width != b_size.width) ) {
            printf("Failed to mul mats: size error: (%d,%d) != (%d,%d)\n", a_size.height, a_size.width, b_size.height, b_size.width);
            return;
        }

        int a_chans = a->get_image()->channels();
        int b_chans = b->get_image()->channels();

        cv::Mat input_matrices[2] = {*(a->get_image()), *(b->get_image())};
        cv::Mat result;
        cv::merge(input_matrices, 2, result);

        image->set_image(&result);
    }

    void extract_channel(data_dict args) {
        opencv_image* image     = (opencv_image*)args[VOP_ARG_SOURCE];
        int channel             = *(int*)args[VOP_ARG_CHANNEL];

        int output_depth = image->get_image()->depth();
        int output_type = CV_MAKETYPE(output_depth, 1);

        cv::Mat new_mat = cv::Mat(image->get_image()->size(), output_type);
        cv::extractChannel(*(image->get_image()), new_mat, channel);
        image->update_image(new_mat);
    }

    void extract_channels(data_dict args) {
    }

    //!SECTION MATRIX ACCESSING


    /////////////////////////////////////
    // SECTION MATHEMATICAL PRIMITIVES //
    /////////////////////////////////////

    void add_mats(data_dict args) {
        opencv_image* image =   (opencv_image*)args[VOP_ARG_SOURCE];
        opencv_image* a =       (opencv_image*)args[VOP_ARG_A];
        opencv_image* b =       (opencv_image*)args[VOP_ARG_B];

        // Check to ensure channel compatibility
        int a_chans = a->get_image()->channels();
        int b_chans = b->get_image()->channels();
        if (a_chans != b_chans) {
            printf("Failed to add mats: channel error: %d != %d\n", a_chans, b_chans);
            return;
        }

        // Check to ensure size compatibility
        cv::Size a_size = a->get_image()->size();
        cv::Size b_size = b->get_image()->size();
        if ( (a_size.height != b_size.height) || (a_size.width != b_size.width) ) {
            printf("Failed to add mats: size error: (%d,%d) != (%d,%d)\n", a_size.height, a_size.width, b_size.height, b_size.width);
            return;
        }

        cv::Mat result = cv::Mat(a->get_image()->size(), a->get_image()->type());
        cv::add(*a->get_image(), *b->get_image(), result, cv::noArray(), a->get_image()->type());
        image->set_image(&result);
    }

    void sub_mats(data_dict args) {
        opencv_image* image =   (opencv_image*)args[VOP_ARG_SOURCE];
        opencv_image* a =       (opencv_image*)args[VOP_ARG_A];
        opencv_image* b =       (opencv_image*)args[VOP_ARG_B];

        // Check to ensure channel compatibility
        int a_chans = a->get_image()->channels();
        int b_chans = b->get_image()->channels();
        if (a_chans != b_chans) {
            printf("Failed to sub mats: channel error: %d != %d\n", a_chans, b_chans);
            return;
        }

        // Check to ensure size compatibility
        cv::Size a_size = a->get_image()->size();
        cv::Size b_size = b->get_image()->size();
        if ( (a_size.height != b_size.height) || (a_size.width != b_size.width) ) {
            printf("Failed to sub mats: size error: (%d,%d) != (%d,%d)\n", a_size.height, a_size.width, b_size.height, b_size.width);
            return;
        }

        cv::Mat result = cv::Mat(a->get_image()->size(), a->get_image()->type());
        cv::subtract(*a->get_image(),*b->get_image(),result, cv::noArray(), a->get_image()->type());
        image->set_image(&result);
    }

    void mul_mats(data_dict args) {
        opencv_image* image =   (opencv_image*)args[VOP_ARG_SOURCE];
        opencv_image* a =       (opencv_image*)args[VOP_ARG_A];
        opencv_image* b =       (opencv_image*)args[VOP_ARG_B];

        // Check to ensure channel compatibility
        int a_chans = a->get_image()->channels();
        int b_chans = b->get_image()->channels();
        if (a_chans != b_chans) {
            printf("Failed to mul mats: channel error: %d != %d\n", a_chans, b_chans);
            return;
        }

        // Check to ensure type compatibility
        int a_type = a->get_image()->type();
        int b_type = b->get_image()->type();
        if (a_type != b_type) {
            printf("Failed to mul mats: type error: %d != %d\n", a_type, b_type);
            return;
        }

        // Check to ensure size compatibility
        cv::Size a_size = a->get_image()->size();
        cv::Size b_size = b->get_image()->size();
        if ( (a_size.height != b_size.height) || (a_size.width != b_size.width) ) {
            printf("Failed to mul mats: size error: (%d,%d) != (%d,%d)\n", a_size.height, a_size.width, b_size.height, b_size.width);
            return;
        }

        cv::Mat result = cv::Mat(a->get_image()->size(), a->get_image()->type());
        cv::multiply(*a->get_image(), *b->get_image(), result, 1.0, a->get_image()->type());
        image->set_image(&result);
    }

    void div_mats(data_dict args) {
        opencv_image* image =   (opencv_image*)args[VOP_ARG_SOURCE];
        opencv_image* a =       (opencv_image*)args[VOP_ARG_A];
        opencv_image* b =       (opencv_image*)args[VOP_ARG_B];

        // Check to ensure channel compatibility
        int a_chans = a->get_image()->channels();
        int b_chans = b->get_image()->channels();
        if (a_chans != b_chans) {
            printf("Failed to div mats: channel error: %d != %d\n", a_chans, b_chans);
            return;
        }

        // Check to ensure size compatibility
        cv::Size a_size = a->get_image()->size();
        cv::Size b_size = b->get_image()->size();
        if ( (a_size.height != b_size.height) || (a_size.width != b_size.width) ) {
            printf("Failed to div mats: size error: (%d,%d) != (%d,%d)\n", a_size.height, a_size.width, b_size.height, b_size.width);
            return;
        }

        cv::Mat result = cv::Mat(a->get_image()->size(), a->get_image()->type());
        cv::divide(*a->get_image(), *b->get_image(), result, 1.0, a->get_image()->type());
        image->set_image(&result);
    }

    void apply_unary_op(data_dict args) {
        opencv_image* image =   (opencv_image*)args[VOP_ARG_SOURCE];
        std::string op = *(std::string*)args[VOP_ARG_OP];

        if (op.compare("negate") == 0){
            image->get_image()->forEach<float>([](float& value, const int* pos) -> void {
                value = value*-1;
            });
        } else if (op.compare("cos") == 0) {
            image->get_image()->forEach<float>([](float& value, const int* pos) -> void {
                value = cos(value*M_PI/180);
            });
        } else if (op.compare("sin") == 0) {
            image->get_image()->forEach<float>([](float& value, const int* pos) -> void {
                value = sin(value*M_PI/180);
            });
        }
    }

    //!SECTION MATHEMATICAL PRIMITIVES

    //////////////////////////////
    // SECTION OBJECT DETECTION //
    //////////////////////////////

    void match_template(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        opencv_image* templ = (opencv_image*)args[VOP_ARG_TEMPLATE];
        int method = *(int*)args[VOP_ARG_METHOD];

        // Check if template is larger than source image in any dimension
        int img_width = image->get_width();
        int img_height = image->get_height();
        int templ_width = templ->get_width();
        int templ_height = templ->get_height();

        // If template is larger than source in any dimension, pad the source image
        if (templ_width > img_width || templ_height > img_height) {
            int new_width = std::max(img_width, templ_width);
            int new_height = std::max(img_height, templ_height);

            // Calculate padding amounts
            int pad_right = std::max(0, templ_width - img_width);
            int pad_bottom = std::max(0, templ_height - img_height);

            // Create a padded version of the source image
            cv::Mat padded_image;
            cv::copyMakeBorder(*image->get_image(), padded_image,
                              0, pad_bottom, 0, pad_right,
                              cv::BORDER_CONSTANT, cv::Scalar(0));

            // Update the source image
            image->set_image(&padded_image);

            // Update dimensions after padding
            img_width = image->get_width();
            img_height = image->get_height();
        }

        int result_cols = img_width - templ_width + 1;
        int result_rows = img_height - templ_height + 1;

        // Ensure we have valid dimensions for result
        if (result_cols <= 0 || result_rows <= 0) {
            // Create empty result with minimum size
            cv::Mat result(1, 1, CV_32FC1, cv::Scalar(0));
            return;
        }

        cv::Mat result;
        result.create(result_rows, result_cols, CV_32FC1);
        cv::matchTemplate(*image->get_image(), *templ->get_image(), result, method);
        // cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

        double minval;
        double maxval;
        cv::Point minloc;
        cv::Point maxloc;

        cv::minMaxLoc(result, &minval, &maxval, &minloc, &maxloc);

        cv::Point top_left;
        cv::Point bottom_right;
        double amount;

        if (method == cv::TM_SQDIFF || method == cv::TM_SQDIFF_NORMED) {
            top_left = minloc;
            amount = minval;
        } else {
            top_left = maxloc;
            amount = maxval;
        }
        bottom_right = cv::Point(top_left.x + templ_width, top_left.y + templ_height);

        cv::rectangle(*(image->get_image()), top_left, bottom_right, cv::Scalar(0, 0, 0, 255), 2, 8, 0);

        *((int*)args[VOP_ARG_X]) = top_left.x;
        *((int*)args[VOP_ARG_Y]) = top_left.y;
        *((int*)args[VOP_ARG_WIDTH]) = templ_width;
        *((int*)args[VOP_ARG_HEIGHT]) = templ_height;
        *((double*)args[VOP_ARG_AMOUNT]) = amount;
    }

    void crop_to_ROI(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        int x = *(int*)args[VOP_ARG_X];
        int y = *(int*)args[VOP_ARG_Y];
        int width = *(int*)args[VOP_ARG_WIDTH];
        int height = *(int*)args[VOP_ARG_HEIGHT];

        cv::Mat result;

        // If the ROI is larger than the image, pad the image
        int img_width = image->get_width();
        int img_height = image->get_height();
        if (x + width > img_width || y + height > img_height) {
            int new_width = std::max(img_width, x + width);
            int new_height = std::max(img_height, y + height);

            // Calculate padding amounts
            int pad_right = std::max(0, new_width - img_width);
            int pad_bottom = std::max(0, new_height - img_height);

            // Create a padded version of the source image
            cv::Mat padded_image;
            cv::copyMakeBorder(*image->get_image(), padded_image,
                              0, pad_bottom, 0, pad_right,
                              cv::BORDER_CONSTANT, cv::Scalar(0));

            // Update the source image
            image->set_image(&padded_image);
        }
        cv::Rect ROI(x, y, width, height);
        result = (*image->get_image())(ROI);
        image->set_image(&result);
    }

    void min_max_loc(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];

        cv::Point minloc;
        cv::Point maxloc;

        cv::minMaxLoc(*image->get_image(), (double*)args[VOP_ARG_MINVAL], (double*)args[VOP_ARG_MAXVAL], &minloc, &maxloc);
        *((int*)args[VOP_ARG_MINLOCX]) = minloc.x;
        *((int*)args[VOP_ARG_MINLOCY]) = minloc.y;
        *((int*)args[VOP_ARG_MAXLOCX]) = maxloc.x;
        *((int*)args[VOP_ARG_MAXLOCY]) = maxloc.y;

        printf("Result min %f at (%d, %d)\n", *((double*)args[VOP_ARG_MINVAL]), minloc.x, minloc.y);
        printf("Result max %f at (%d, %d)\n", *((double*)args[VOP_ARG_MAXVAL]), maxloc.x, maxloc.y);
    }

    void get_object(data_dict args) {
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];
        OBJ_REP_TYPE* object = (OBJ_REP_TYPE*)args[VOP_ARG_OBJECT];

        object->update_image(source);
        if (source->is_empty()) {
            object->set_object_null(true);
        }
        #ifdef ENABLE_TORCH
        source->update_image(object->get_base_image());
        #else
        source->update_image(object->get_object_image()(object->get_mask_bbox()));
        #endif
    }

    #ifdef ENABLE_TORCH
    void extract_visible(data_dict args) {
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];
        OBJ_REP_TYPE* query = (OBJ_REP_TYPE*)args[VOP_ARG_QUERY];
        OBJ_REP_TYPE* base = (OBJ_REP_TYPE*)args[VOP_ARG_BASE];

        token_sequence* query_tokens = query->get_tokens();
        token_sequence* base_tokens = base->get_tokens();
        if (query_tokens == NULL || base_tokens == NULL) {
            throw std::runtime_error("Query or base object does not have JEPA tokens.");
        }

        token_sequence* out_tokens = new token_sequence();
        vltm->get_vcd_model()->extract(query_tokens, base_tokens, out_tokens);

        OBJ_REP_TYPE* object = (OBJ_REP_TYPE*)args[VOP_ARG_OBJECT];
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];
        object->update_jepa_model(vltm->get_vcd_model());
        object->update_tokens(out_tokens);
        source->update_image(object->get_base_image());
    }

    void deobscure_object(data_dict args) {
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];
        OBJ_REP_TYPE* template_obj = (OBJ_REP_TYPE*)args[VOP_ARG_TEMPLATE];
        OBJ_REP_TYPE* query = (OBJ_REP_TYPE*)args[VOP_ARG_QUERY];

        token_sequence* template_tokens = template_obj->get_tokens();
        token_sequence* query_tokens = query->get_tokens();
        if (template_tokens == NULL || query_tokens == NULL) {
            throw std::runtime_error("Template or query object does not have JEPA tokens.");
        }

        token_sequence* out_tokens = new token_sequence();
        vltm->get_vcd_model()->deobscure(template_tokens, query_tokens, out_tokens);

        OBJ_REP_TYPE* object = (OBJ_REP_TYPE*)args[VOP_ARG_OBJECT];
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];
        object->update_jepa_model(vltm->get_vcd_model());
        object->update_tokens(out_tokens);
        source->update_image(object->get_base_image());
    }

    void extract_full_object(data_dict args) {
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];
        OBJ_REP_TYPE* query = (OBJ_REP_TYPE*)args[VOP_ARG_QUERY];
        OBJ_REP_TYPE* base = (OBJ_REP_TYPE*)args[VOP_ARG_BASE];

        token_sequence* query_tokens = query->get_tokens();
        token_sequence* base_tokens = base->get_tokens();
        if (query_tokens == NULL || base_tokens == NULL) {
            throw std::runtime_error("Query or base object does not have JEPA tokens.");
        }

        token_sequence* partial_tokens = new token_sequence();
        vltm->get_vcd_model()->extract(query_tokens, base_tokens, partial_tokens);

        token_sequence* out_tokens = new token_sequence();
        vltm->get_vcd_model()->deobscure(query_tokens, partial_tokens, out_tokens);

        OBJ_REP_TYPE* object = (OBJ_REP_TYPE*)args[VOP_ARG_OBJECT];
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];
        object->update_jepa_model(vltm->get_vcd_model());
        object->update_tokens(out_tokens);
        source->update_image(object->get_base_image());
    }

    #else
    void extract_visible(data_dict args) {
        printf("Error: extract_visible not implemented without Torch support.\n");
    }
    void deobscure_object(data_dict args) {
        printf("Error: deobscure_object not implemented without Torch support.\n");
    }
    void extract_full_object(data_dict args) {
        printf("Error: extract_full_object not implemented without Torch support.\n");
    }
    #endif

    void object_distance(data_dict args) {
        OBJ_REP_TYPE* query = (OBJ_REP_TYPE*)args[VOP_ARG_QUERY];
        OBJ_REP_TYPE* target = (OBJ_REP_TYPE*)args[VOP_ARG_TARGET];
        double* distance = (double*)args[VOP_ARG_DISTANCE];
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];

        *distance = query->get_shape_distance(target);
        source->update_image(query->get_base_image());
    }

    void segment(data_dict args) {
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];
        std::string method;
        if (args[VOP_ARG_METHOD] == NULL) {
            method = std::string("colors");
        } else {
            method = *(std::string*)args[VOP_ARG_METHOD];
        }
        std::vector<opencv_image*>* segments = (std::vector<opencv_image*>*)args[VOP_ARG_SEGMENTS];

        std::vector<cv::Mat> segment_masks;
        if (method.compare("watershed") == 0) {
            int num_segments = __segment_image_watershed(*source->get_image(), segment_masks);
        } else if (method.compare("colors") == 0) {
            int num_segments = __segment_image_colors(*source->get_image(), segment_masks);
        } else {
            printf("Invalid segmentation method: %s\n", method.c_str());
            return;
        }

        for (int i = 0; i < segment_masks.size(); i++) {
            cv::Mat masked_image;
            cv::bitwise_and(*source->get_image(), *source->get_image(), masked_image, segment_masks[i]);
            opencv_image* segment = new opencv_image();
            masked_image.convertTo(masked_image, CV_32F);
            segment->update_image(masked_image);
            segments->push_back(segment);
        }

        *((int*)args[VOP_ARG_COUNT]) = segments->size();
    }

    int __segment_image_colors(cv::Mat image, std::vector<cv::Mat> &masks) {
        // Convert the image from CV_32FC4 to CV_8UC4
        cv::Mat image_copy;
        image.copyTo(image_copy);
        image_copy.convertTo(image_copy, CV_8UC4, 255.0);

        // Define the HSV ranges for each color
        cv::Scalar lower_red = cv::Scalar(0, 100, 100);
        cv::Scalar upper_red = cv::Scalar(10, 255, 255);
        cv::Scalar lower_green = cv::Scalar(50, 100, 100);
        cv::Scalar upper_green = cv::Scalar(70, 255, 255);
        cv::Scalar lower_blue = cv::Scalar(110, 100, 100);
        cv::Scalar upper_blue = cv::Scalar(130, 255, 255);
        cv::Scalar lower_cyan = cv::Scalar(80, 100, 100);
        cv::Scalar upper_cyan = cv::Scalar(100, 255, 255);
        cv::Scalar lower_magenta = cv::Scalar(140, 100, 100);
        cv::Scalar upper_magenta = cv::Scalar(160, 255, 255);
        cv::Scalar lower_yellow = cv::Scalar(20, 100, 100);
        cv::Scalar upper_yellow = cv::Scalar(40, 255, 255);
        cv::Scalar lower_black = cv::Scalar(0, 0, 20);
        cv::Scalar upper_black = cv::Scalar(180, 255, 50);
        cv::Scalar lower_white = cv::Scalar(0, 0, 200);
        cv::Scalar upper_white = cv::Scalar(180, 50, 255);

        // Convert the image to HSV
        cv::Mat image_hsv;
        cv::cvtColor(image_copy, image_hsv, cv::COLOR_RGB2HSV);

        // Generate the masks for each color
        cv::Mat mask_red;
        cv::inRange(image_hsv, lower_red, upper_red, mask_red);
        cv::Mat mask_green;
        cv::inRange(image_hsv, lower_green, upper_green, mask_green);
        cv::Mat mask_blue;
        cv::inRange(image_hsv, lower_blue, upper_blue, mask_blue);
        cv::Mat mask_cyan;
        cv::inRange(image_hsv, lower_cyan, upper_cyan, mask_cyan);
        cv::Mat mask_magenta;
        cv::inRange(image_hsv, lower_magenta, upper_magenta, mask_magenta);
        cv::Mat mask_yellow;
        cv::inRange(image_hsv, lower_yellow, upper_yellow, mask_yellow);
        cv::Mat mask_black;
        cv::inRange(image_hsv, lower_black, upper_black, mask_black);
        cv::Mat mask_white;
        cv::inRange(image_hsv, lower_white, upper_white, mask_white);

        // Add the masks to the masks vector if they are not empty
        if (cv::countNonZero(mask_red) > 16) {
            masks.push_back(mask_red);
        }
        if (cv::countNonZero(mask_green) > 16) {
            masks.push_back(mask_green);
        }
        if (cv::countNonZero(mask_blue) > 16) {
            masks.push_back(mask_blue);
        }
        if (cv::countNonZero(mask_cyan) > 16) {
            masks.push_back(mask_cyan);
        }
        if (cv::countNonZero(mask_magenta) > 16) {
            masks.push_back(mask_magenta);
        }
        if (cv::countNonZero(mask_yellow) > 16) {
            masks.push_back(mask_yellow);
        }
        if (cv::countNonZero(mask_black) > 16) {
            masks.push_back(mask_black);
        }
        if (cv::countNonZero(mask_white) > 16) {
            masks.push_back(mask_white);
        }

        return masks.size();
    }

    int __segment_image_watershed(cv::Mat image, std::vector<cv::Mat> &masks) {
        // get_edges()
        // Get inverted edge mask
        cv::Mat image_copy;
        image.copyTo(image_copy);
        image_copy.convertTo(image_copy, CV_8U, 255.0);
        cv::Mat blurred_img;
        cv::blur(image_copy, blurred_img, cv::Size(3, 3), cv::Point(-1, -1));
        cv::Mat detected_edges;
        cv::Canny(blurred_img, detected_edges, 75, 75 * 4, 3);
        cv::Mat edge_mask;
        edge_mask = detected_edges == 0;
        // Multiply it by the alpha channel of the image to mask background regions
        cv::Mat image_alpha_mask;
        cv::extractChannel(image_copy, image_alpha_mask, 3);
        image_alpha_mask = image_alpha_mask > 0;
        edge_mask = edge_mask.mul(image_alpha_mask);

        // get_regions()
        // Generate the seed and unknown regions for the watershed algorithm
        cv::Mat seed_regions;
        cv::Mat unknown_regions;
        cv::erode(edge_mask, seed_regions, cv::Mat(), cv::Point(-1, -1), 1);
        cv::dilate(edge_mask, unknown_regions, cv::Mat(), cv::Point(-1, -1), 3);
        unknown_regions = unknown_regions - edge_mask;
        cv::dilate(unknown_regions, unknown_regions, cv::Mat(), cv::Point(-1, -1), 1);

        // get_watershed_markers()
        // Apply the watershed algorithm to segment the image
        cv::Mat watershed_markers;
        int num_masks = cv::connectedComponents(seed_regions, watershed_markers, 8, CV_32S, cv::CCL_DEFAULT);
        watershed_markers += 1;
        watershed_markers.setTo(0, unknown_regions);

        cv::Mat image_copy_flat;
        if (image_copy.channels() == 4) {
            cv::cvtColor(image_copy, image_copy_flat, cv::COLOR_RGBA2RGB);
        } else {
            image_copy.copyTo(image_copy_flat);
        }
        cv::watershed(image_copy_flat, watershed_markers);

        // get_watershed_object_masks()
        // Extract the object masks from the watershed markers
        for (int i = 1; i <= num_masks; i++) {
            cv::Mat mask = cv::Mat::zeros(image_copy.size(), CV_8UC1);
            cv::Mat marker_mask = watershed_markers == i;
            mask.setTo(255, watershed_markers == i);
            masks.push_back(mask);
        }

        return num_masks;
    }

    void get_segment(data_dict args) {
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];
        std::vector<opencv_image*>* segments = (std::vector<opencv_image*>*)args[VOP_ARG_SEGMENTS];
        int index = *(int*)args[VOP_ARG_INDEX];

        source->update_image(*segments->at(index)->get_image());
    }

    void get_affine_from_corners(data_dict args) {
        OBJ_REP_TYPE* query = (OBJ_REP_TYPE*)args[VOP_ARG_QUERY];
        OBJ_REP_TYPE* target = (OBJ_REP_TYPE*)args[VOP_ARG_TARGET];
        visual_ops::affine_transform_struct* transform = (visual_ops::affine_transform_struct*)args[VOP_ARG_TRANSFORM];
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];

        cv::Mat affine_transform = query->get_corner_affine_transform(target);
        transform->affine_transform = new cv::Mat(affine_transform);

        cv::Mat transformed_mask;
        cv::Mat mask_intersection;
        cv::Mat mask_union;
        cv::warpAffine(query->get_mask(), transformed_mask, affine_transform, target->get_mask().size(), cv::INTER_NEAREST);
        cv::bitwise_and(transformed_mask, target->get_mask(), mask_intersection);
        double intersection_area = static_cast<double>(cv::countNonZero(mask_intersection));
        cv::bitwise_or(transformed_mask, target->get_mask(), mask_union);
        double union_area = static_cast<double>(cv::countNonZero(mask_union));
        transform->score = intersection_area / union_area;

        std::pair<int, int> translation = transform->translation_from_affine();
        transform->x = translation.first;
        transform->y = translation.second;

        transform->rotation = transform->rotation_from_affine();

        std::pair<double, double> scale = transform->scale_from_affine();
        transform->scale_x = scale.first;
        transform->scale_y = scale.second;

        source->update_image(transform->show_transform(query, target));
    }

    void get_transforms(data_dict args) {
        OBJ_REP_TYPE* query = (OBJ_REP_TYPE*)args[VOP_ARG_QUERY];
        OBJ_REP_TYPE* target = (OBJ_REP_TYPE*)args[VOP_ARG_TARGET];
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];

        std::vector<visual_ops::affine_transform_struct*>* transforms = (std::vector<visual_ops::affine_transform_struct*>*)args[VOP_ARG_TRANSFORMS];
        transforms->clear();

        std::vector<std::pair<double, cv::Mat*>>* results = query->get_best_affine_transforms(target, -1);

        for (int i = 0; i < results->size(); i++) {
            visual_ops::affine_transform_struct* transform = new visual_ops::affine_transform_struct();
            transform->score = results->at(i).first;
            transform->affine_transform = results->at(i).second;

            std::pair<int, int> translation = transform->translation_from_affine();
            transform->x = translation.first;
            transform->y = translation.second;

            transform->rotation = transform->rotation_from_affine();

            std::pair<double, double> scale = transform->scale_from_affine();
            transform->scale_x = scale.first;
            transform->scale_y = scale.second;

            transforms->push_back(transform);
        }

        source->update_image(query->get_base_image());
        *((int*)args[VOP_ARG_COUNT]) = results->size();
    }

    void get_objects_iou(data_dict args) {
        OBJ_REP_TYPE* a = (OBJ_REP_TYPE*)args[VOP_ARG_A];
        OBJ_REP_TYPE* b = (OBJ_REP_TYPE*)args[VOP_ARG_B];
        double* iou = (double*)args[VOP_ARG_IOU];
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];

        cv::Mat a_mask = a->get_cropped_mask();
        cv::Mat b_mask = b->get_cropped_mask();

        cv::Mat a_full_sized;
        cv::Mat b_full_sized;
        cv::Size2d merged_size = cv::Size2d(std::max(a_mask.cols, b_mask.cols), std::max(a_mask.rows, b_mask.rows));
        int merged_width = static_cast<int>(merged_size.width);
        int merged_height = static_cast<int>(merged_size.height);

        int border_top_bottom_a = merged_height - a_mask.rows;
        int border_left_right_a  = merged_width - a_mask.cols;
        int border_bottom_a = border_top_bottom_a / 2;
        int border_top_a = border_top_bottom_a - border_bottom_a;
        int border_right_a  = border_left_right_a / 2;
        int border_left_a   = border_left_right_a - border_right_a;
        cv::copyMakeBorder(a_mask, a_full_sized, border_top_a, border_bottom_a, border_left_a, border_right_a, cv::BORDER_CONSTANT, cv::Scalar(0));

        int border_top_bottom_b = merged_height - b_mask.rows;
        int border_left_right_b  = merged_width - b_mask.cols;
        int border_bottom_b = border_top_bottom_b / 2;
        int border_top_b = border_top_bottom_b - border_bottom_b;
        int border_right_b  = border_left_right_b / 2;
        int border_left_b   = border_left_right_b - border_right_b;
        cv::copyMakeBorder(b_mask, b_full_sized, border_top_b, border_bottom_b, border_left_b, border_right_b, cv::BORDER_CONSTANT, cv::Scalar(0));

        cv::Mat mask_intersection;
        cv::Mat mask_union;
        cv::bitwise_and(a_full_sized, b_full_sized, mask_intersection);
        cv::dilate(mask_intersection, mask_intersection, cv::Mat(), cv::Point(-1, -1), 1);
        cv::bitwise_or(a_full_sized, b_full_sized, mask_union);

        double intersection_area = static_cast<double>(cv::countNonZero(mask_intersection));
        double union_area = static_cast<double>(cv::countNonZero(mask_union));
        *iou = intersection_area / union_area;

        cv::Mat mask_iou;
        cv::add(mask_intersection, mask_union, mask_iou, cv::noArray(), CV_32F);
        mask_iou = mask_iou / 2.0;
        mask_iou.convertTo(mask_iou, CV_8U);

        source->update_image(mask_iou);
    }

    void get_object_coverage(data_dict args) {
        OBJ_REP_TYPE* a = (OBJ_REP_TYPE*)args[VOP_ARG_A];
        OBJ_REP_TYPE* b = (OBJ_REP_TYPE*)args[VOP_ARG_B];
        int crop;
        double* coverage = (double*)args[VOP_ARG_COVERAGE];
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];

        if (args[VOP_ARG_CROP] == NULL) {
            crop = 1;
        } else {
            crop = *(int*)args[VOP_ARG_CROP];
        }

        cv::Mat a_mask;
        cv::Mat b_mask;
        if (crop == 1) {
            a_mask = a->get_cropped_mask();
            b_mask = b->get_cropped_mask();
        } else {
            a_mask = a->get_mask();
            b_mask = b->get_mask();
        }

        cv::Mat a_full_sized;
        cv::Mat b_full_sized;
        cv::Size2d merged_size = cv::Size2d(std::max(a_mask.cols, b_mask.cols), std::max(a_mask.rows, b_mask.rows));
        int merged_width = static_cast<int>(merged_size.width);
        int merged_height = static_cast<int>(merged_size.height);

        int border_top_bottom_a = merged_height - a_mask.rows;
        int border_left_right_a  = merged_width - a_mask.cols;
        int border_bottom_a = border_top_bottom_a / 2;
        int border_top_a = border_top_bottom_a - border_bottom_a;
        int border_right_a  = border_left_right_a / 2;
        int border_left_a   = border_left_right_a - border_right_a;
        cv::copyMakeBorder(a_mask, a_full_sized, border_top_a, border_bottom_a, border_left_a, border_right_a, cv::BORDER_CONSTANT, cv::Scalar(0));

        int border_top_bottom_b = merged_height - b_mask.rows;
        int border_left_right_b  = merged_width - b_mask.cols;
        int border_bottom_b = border_top_bottom_b / 2;
        int border_top_b = border_top_bottom_b - border_bottom_b;
        int border_right_b  = border_left_right_b / 2;
        int border_left_b   = border_left_right_b - border_right_b;
        cv::copyMakeBorder(b_mask, b_full_sized, border_top_b, border_bottom_b, border_left_b, border_right_b, cv::BORDER_CONSTANT, cv::Scalar(0));
        cv::dilate(b_full_sized, b_full_sized, cv::Mat(), cv::Point(-1, -1), 1);

        cv::Mat mask_union;
        cv::bitwise_or(a_full_sized, b_full_sized, mask_union);

        cv::Mat uncovered_pixels = mask_union - b_full_sized;
        double uncovered_area = static_cast<double>(cv::countNonZero(uncovered_pixels));
        double a_area = static_cast<double>(cv::countNonZero(a_full_sized));

        *coverage = 1.0 - (uncovered_area / a_area);

        cv::Mat mask_coverage;
        cv::add(uncovered_pixels, mask_union, mask_coverage, cv::noArray(), CV_32F);
        mask_coverage = mask_coverage / 2.0;
        mask_coverage.convertTo(mask_coverage, CV_8U);
        source->update_image(mask_coverage);
    }

    //!SECTION OBJECT DETECTION

    ///////////////////////////////////
    // SECTION ENCODING AND DECODING //
    ///////////////////////////////////

    #ifdef ENABLE_TORCH
    void encode(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        // TODO: Make this more flexible and selectable at runtime
        // latent_representation* latent = (latent_representation*)args[VOP_ARG_LATENT];
        token_sequence* latent = (token_sequence*)args[VOP_ARG_LATENT];
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];

        vltm->encode_image(image, latent);
    }

    void decode(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        // TODO: Make this more flexible and selectable at runtime
        // latent_representation* latent =
        // (latent_representation*)args[VOP_ARG_LATENT];
        token_sequence* latent = (token_sequence*)args[VOP_ARG_LATENT];
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];

        vltm->decode_representation(latent, image);
    }
    #else
    void encode(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        opencv_image* latent = (opencv_image*)args[VOP_ARG_LATENT];
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];

        latent->update_image(*image->get_image());
    }

    void decode(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        opencv_image* latent = (opencv_image*)args[VOP_ARG_LATENT];
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];

        image->update_image(*latent->get_image());
    }
    #endif

    //!SECTION ENCODING AND DECODING

    ///////////////////////////////////
    // SECTION VLTM-BASED OPERATIONS //
    ///////////////////////////////////

    void recognize(data_dict args) {
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];

        vmem_match** matches = new vmem_match*[3];
        #ifdef ENABLE_TORCH
        // TODO: Make this more flexible and selectable at runtime
        // latent_representation* latent = new latent_representation();
        token_sequence* latent = new token_sequence();
        vltm->encode_image(source, latent);
        vltm->match(latent, matches, 3);
        #else
        vltm->match(source, matches, 3);
        #endif

        ((std::string*)args[VOP_ARG_CLASS1])->assign(matches[0]->entity_id);
        ((std::string*)args[VOP_ARG_CLASS2])->assign(matches[1]->entity_id);
        ((std::string*)args[VOP_ARG_CLASS3])->assign(matches[2]->entity_id);

        // printf("Conf1: %f\n", matches[0]->confidence);
        *((double*)args[VOP_ARG_CONF1]) = matches[0]->confidence;
        *((double*)args[VOP_ARG_CONF2]) = matches[1]->confidence;
        *((double*)args[VOP_ARG_CONF3]) = matches[2]->confidence;
        // printf("Arg Conf1: %f\n", *((double*)args[VOP_ARG_CONF1]));

    }

    void learn_from(data_dict args) {
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];
        std::string class_name = *(std::string*)args[VOP_ARG_CLASSNAME];
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];

        #ifdef ENABLE_TORCH
        // TODO: Make this more flexible and selectable at runtime
        // latent_representation* latent = new latent_representation();
        token_sequence* latent = new token_sequence();
        vltm->encode_image(source, latent);
        vltm->store_percept(latent, class_name);
        #else
        vltm->store_percept(source, class_name);
        #endif
    }

    void generate(data_dict args) {
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];
        std::string class_name = *(std::string*)args[VOP_ARG_CLASSNAME];
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];

        #ifdef ENABLE_TORCH
        // TODO: Make this more flexible and selectable at runtime
        // latent_representation* latent = new latent_representation();
        token_sequence* latent = new token_sequence();
        vltm->recall(class_name, latent);
        vltm->decode_representation(latent, source);
        #else
        vltm->recall(class_name, source);
        #endif
    }

    //!SECTION VLTM-BASED OPERATIONS
} // namespace visual_ops
