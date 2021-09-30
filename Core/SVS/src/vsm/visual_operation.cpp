#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "visual_sensory_memory.h"
#include "visual_operation.h"
#include "image.h"


namespace visual_ops
{
    ///////////////////
    // IMAGE SOURCES //
    ///////////////////

    void get_from_vsm(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        int buffer_index = *(int*)args[ARG_BUFFERINDEX];
        visual_sensory_memory* vsm = (visual_sensory_memory*)args[ARG_VSM];

        image->copy_from(vsm->get_vision(buffer_index));
        cv::Size image_shape = image->get_image()->size();
        // printf("VSM image shape: (%d, %d, %d), type: %d\n", image_shape.width, image_shape.height, image->get_image()->channels(), image->get_image()->type());
    }

    void load_from_file(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        std::string* filepath = (std::string*)args[ARG_FILEPATH];
        
        image->update_image(cv::imread(*filepath, cv::IMREAD_UNCHANGED));
        cv::Size image_shape = image->get_image()->size();
    }

    void save_to_file(data_dict args) {
        cv::Mat image = *(((opencv_image*)args[ARG_TARGET])->get_image());
        std::string filepath = *(std::string*)args[ARG_FILEPATH];
 
        cv::Size image_shape = image.size();

        cv::Mat converted_image;
        image.convertTo(converted_image, CV_8UC3, 255);
        
        try {
            cv::imwrite(filepath, converted_image);
        } catch (const cv::Exception& ex) {
            printf("Exception converting image to file: %s\n", ex.what());
        }
        
    }

    void display_image(data_dict args) {
        cv::Mat image = *(((opencv_image*)args[ARG_TARGET])->get_image());
        std::string window_name = *(std::string*)args[ARG_WINDOWNAME];

        cv::imshow(window_name, image);
        cv::waitKey(1);
    }


    ////////////////////////////
    // VISUAL TRANSFORMATIONS //
    ////////////////////////////

    void identity(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];

        cv::Mat result;
        result = *(image->get_image());
        image->set_image(&result);
    }

    void blur(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        cv::Size ksize = *(cv::Size*)args[ARG_KSIZE];
        
        cv::Point anchor;
        if (args[ARG_ANCHOR] != NULL) {
            anchor = *(cv::Point*)args[ARG_ANCHOR];
        } else {
            anchor = cv::Point(-1, -1);
        }

        int borderType;
        if (args[ARG_BORDERTYPE] != NULL) {
            borderType = *(int*)args[ARG_BORDERTYPE];
        } else {
            borderType = cv::BORDER_DEFAULT;
        }

        cv::Mat result;
        cv::blur(*image->get_image(), result, ksize, anchor, borderType);
        image->set_image(&result);
    }

    void GaussianBlur(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        cv::Size ksize = *(cv::Size*)args[ARG_KSIZE];
        double sigmaX = *(double*)args[ARG_SIGMAX];
        double sigmaY = *(double*)args[ARG_SIGMAY];
        int borderType;
        if (args[ARG_BORDERTYPE] != NULL) {
            borderType = *(int*)args[ARG_BORDERTYPE];
        } else {
            borderType = cv::BORDER_DEFAULT;
        }

        cv::Mat result;
        cv::GaussianBlur(*image->get_image(), result, ksize, sigmaX, sigmaY, borderType);
        image->set_image(&result);
    }

    void greyscale(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];

        cv::Mat result;
        cv::cvtColor(*image->get_image(), result, cv::COLOR_BGRA2GRAY);
        image->set_image(&result);
    }

    void threshold(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        double thresh = *(double*)args[ARG_THRESH];
        double maxval = *(double*)args[ARG_MAXVAL];
        int type = *(int*)args[ARG_TYPE];

        cv::Mat result;
        cv::threshold(*image->get_image(), result, thresh, maxval, type);
        image->set_image(&result);
    }

    void crop_to_ROI(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        int x = *(int*)args[ARG_X];
        int y = *(int*)args[ARG_Y];
        int width = *(int*)args[ARG_WIDTH];
        int height = *(int*)args[ARG_HEIGHT];

        cv::Mat result;
        cv::Rect ROI(x, y, width, height);
        result = (*image->get_image())(ROI);
        image->set_image(&result);
    }


    //////////////////////
    // OBJECT DETECTION //
    //////////////////////

    void match_template(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        opencv_image* templ = (opencv_image*)args[ARG_TEMPLATE];
        int method = *(int*)args[ARG_METHOD];

        int result_cols = image->get_width() - templ->get_width() + 1;
        int result_rows = image->get_height() - templ->get_height() + 1;
        cv::Mat result;
        result.create( result_rows, result_cols, CV_32FC1 );
        cv::matchTemplate(*image->get_image(), *templ->get_image(), result, method);
        normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

        image->set_image(&result);
    }

    void min_max_loc(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        double* minval = (double*)args[ARG_MINVAL];
        double* maxval = (double*)args[ARG_MAXVAL];
        cv::Point* minloc = (cv::Point*)args[ARG_MINLOC];
        cv::Point* maxloc = (cv::Point*)args[ARG_MAXLOC];
        cv::minMaxLoc(*image->get_image(), minval, maxval, minloc, maxloc);
    }
} // namespace visual_ops
