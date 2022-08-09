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
        opencv_image* image;
        image = (opencv_image*)args[VOP_ARG_TARGET];

        int buffer_index;
        if (args[VOP_ARG_BUFFERINDEX] == NULL) {
            buffer_index = 0;
        } else {
            buffer_index = *(int*)args[VOP_ARG_BUFFERINDEX];
        }

        visual_sensory_memory* vsm;
        vsm = (visual_sensory_memory*)args[VOP_ARG_VSM];

        image->copy_from(vsm->get_vision(buffer_index));
        cv::Size image_shape = image->get_image()->size();
        // printf("VSM image shape: (%d, %d, %d), type: %d\n", image_shape.width, image_shape.height, image->get_image()->channels(), image->get_image()->type());
    }

    void load_from_file(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_TARGET];
        std::string* filepath = (std::string*)args[VOP_ARG_FILEPATH];
        
        image->update_image(cv::imread(*filepath, cv::IMREAD_UNCHANGED));
        cv::Size image_shape = image->get_image()->size();
    }

    void save_to_file(data_dict args) {
        cv::Mat image = *(((opencv_image*)args[VOP_ARG_TARGET])->get_image());
        std::string filepath = *(std::string*)args[VOP_ARG_FILEPATH];
        
        try {
            cv::imwrite(filepath, image);
        } catch (const cv::Exception& ex) {
            printf("Exception converting image to file: %s\n", ex.what());
        }
        
    }

    void display_image(data_dict args) {
        cv::Mat image = *(((opencv_image*)args[VOP_ARG_TARGET])->get_image());
        std::string window_name = *(std::string*)args[VOP_ARG_WINDOWNAME];

        cv::imshow(window_name, image);
        cv::waitKey(1);
    }


    ////////////////////////////
    // VISUAL TRANSFORMATIONS //
    ////////////////////////////

    void identity(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_TARGET];

        cv::Mat result;
        result = *(image->get_image());
        image->set_image(&result);
    }

    void blur(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_TARGET];
        int size_x = *(int*)args[VOP_ARG_SIZEX];
        int size_y = *(int*)args[VOP_ARG_SIZEX];
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
        opencv_image* image = (opencv_image*)args[VOP_ARG_TARGET];

        int size_x = *(int*)args[VOP_ARG_SIZEX];
        int size_y = *(int*)args[VOP_ARG_SIZEX];
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
        opencv_image* image = (opencv_image*)args[VOP_ARG_TARGET];

        cv::Mat result;
        cv::cvtColor(*image->get_image(), result, cv::COLOR_BGRA2GRAY);
        image->set_image(&result);
    }

    void threshold(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_TARGET];
        double thresh = *(double*)args[VOP_ARG_THRESH];
        double maxval = *(double*)args[VOP_ARG_MAXVAL];
        int type = *(int*)args[VOP_ARG_TYPE];

        cv::Mat result;
        cv::threshold(*image->get_image(), result, thresh, maxval, type);
        image->set_image(&result);
    }
   
    
    //////////////////////
    // OBJECT DETECTION //
    //////////////////////

    void match_template(data_dict args) {
        opencv_image* image = (opencv_image*)args[VOP_ARG_TARGET];
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
        opencv_image* image = (opencv_image*)args[VOP_ARG_TARGET];
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
        opencv_image* image = (opencv_image*)args[VOP_ARG_TARGET];

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
} // namespace visual_ops
