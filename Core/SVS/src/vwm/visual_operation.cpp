#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "visual_operation.h"
#include "visual_input_buffer.h"
#include "visual_long_term_memory.h"
#include "exact_visual_concept_descriptor.h"
#include "vae_visual_concept_descriptor.h"
#include "image.h"
#include "latent_representation.h"


namespace visual_ops
{
    ///////////////////
    // IMAGE SOURCES //
    ///////////////////

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


    ////////////////////////////
    // VISUAL TRANSFORMATIONS //
    ////////////////////////////

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
        if (args[VOP_ARG_ANCHORX] != NULL) {
            anchor.x = -1;
        } else {
            anchor.x = *(int*)args[VOP_ARG_ANCHORX];
        }

        if (args[VOP_ARG_ANCHORY] != NULL) {
            anchor.y = -1;
        } else {
            anchor.y = *(int*)args[VOP_ARG_ANCHORY];
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
        float center_x, center_y;
        center_x = (float)image->get_width() / 2.0;
        center_y = (float)image->get_height() / 2.0;
        rot_mat = cv::getRotationMatrix2D(cv::Point2f(center_x, center_y), amount, 1.0);
        cv::warpAffine(*(image->get_image()), result, rot_mat, image->get_image()->size());
        image->set_image(&result);
    }

    /////////////////////
    // MATRIX CREATION //
    /////////////////////

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

    //////////////////////
    // MATRIX ACCESSING //
    //////////////////////

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


    /////////////////////////////
    // MATHEMATICAL PRIMITIVES //
    /////////////////////////////

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

    //////////////////////
    // OBJECT DETECTION //
    //////////////////////

    void match_template(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        opencv_image* templ = (opencv_image*)args[VOP_ARG_TEMPLATE];
        int method = *(int*)args[VOP_ARG_METHOD];

        int result_cols = image->get_width() - templ->get_width() + 1;
        int result_rows = image->get_height() - templ->get_height() + 1;
        cv::Mat result;
        result.create( result_rows, result_cols, CV_32FC1 );
        cv::matchTemplate(*image->get_image(), *templ->get_image(), result, method);
        normalize( result, result, 0.0, 1.0, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );

        double min, max;
        cv::Point minloc;
        cv::Point maxloc;

        cv::minMaxLoc(result, &min, &max, &minloc, &maxloc);


        image->set_image(&result);
    }

    void crop_to_ROI(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        int x = *(int*)args[VOP_ARG_X];
        int y = *(int*)args[VOP_ARG_Y];
        int width = *(int*)args[VOP_ARG_WIDTH];
        int height = *(int*)args[VOP_ARG_HEIGHT];

        cv::Mat result;
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

    ///////////////////////////
    // ENCODING AND DECODING //
    ///////////////////////////

    #ifdef ENABLE_TORCH
    void encode(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        latent_representation* latent = (latent_representation*)args[VOP_ARG_LATENT];
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];

        vltm->encode_image(image, latent);
    }

    void decode(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_SOURCE];
        latent_representation* latent = (latent_representation*)args[VOP_ARG_LATENT];
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];

        vltm->decode_latent(latent, image);
    }
    #endif

    ///////////////////////////
    // VLTM-BASED OPERATIONS //
    ///////////////////////////

    void recognize(data_dict args) {
        opencv_image* source = (opencv_image*)args[VOP_ARG_SOURCE];
        VLTM_TYPE* vltm = (VLTM_TYPE*)args[VOP_ARG_VLTM];

        vmem_match** matches = new vmem_match*[3];
        #ifdef ENABLE_TORCH
        latent_representation* latent = new latent_representation();
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
        latent_representation* latent = new latent_representation();
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
        latent_representation* latent = new latent_representation();
        vltm->recall(class_name, latent);
        vltm->decode_latent(latent, source);
        #else
        vltm->recall(class_name, source);
        #endif
    }
} // namespace visual_ops
