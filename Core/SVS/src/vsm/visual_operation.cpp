#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "visual_operation.h"
#include "image.h"


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

namespace visual_ops
{
    ///////////////////
    // IMAGE SOURCES //
    ///////////////////
    void load_from_file(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        std::string* filepath = (std::string*)args[ARG_FILEPATH];

        image->update_image(cv::imread(*filepath, cv::IMREAD_COLOR));
    }

    void save_to_file(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        std::string filepath = *(std::string*)args[ARG_FILEPATH];

        cv::imwrite(filepath, *image->get_image());
    }

    ////////////////////////////
    // VISUAL TRANSFORMATIONS //
    ////////////////////////////
    void identity(data_dict args) {}

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
        cv::blur(*image->get_image(), *image->get_image(), ksize, anchor, borderType);
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
        cv::GaussianBlur(*image->get_image(), *image->get_image(), ksize, sigmaX, sigmaY, borderType);
    }

    void greyscale(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        cv::cvtColor(*image->get_image(), *image->get_image(), cv::COLOR_BGR2GRAY);
    }

    void threshold(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        double thresh = *(double*)args[ARG_THRESH];
        double maxval = *(double*)args[ARG_MAXVAL];
        int type = *(int*)args[ARG_TYPE];
        cv::threshold(*image->get_image(), *image->get_image(), thresh, maxval, type);
    }

    void match_template(data_dict args) {
        opencv_image* image = (opencv_image*)args[ARG_TARGET];
        opencv_image* templ = (opencv_image*)args[ARG_TEMPLATE];
        int method = *(int*)args[ARG_METHOD];
        cv::matchTemplate(*image->get_image(), *templ->get_image(), *image->get_image(), method);
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
