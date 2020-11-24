// OpenCV library
#ifdef ENABLE_OPENCV
#include <opencv2/opencv.hpp>
#endif
// SVS Includes
#include "image.h"
#include "vision_operations.h"


#ifdef ENABLE_OPENCV
template<> void rotate_90(opencv_image& visual);
template<> void rotate_180(opencv_image& visual);
template<> void rotate_270(opencv_image& visual);

template<> void rotate_90(opencv_image& visual) {
    cv::Mat* image = visual.get_image();
    cv::rotate(*image, *image, cv::ROTATE_90_CLOCKWISE);
}

template<> void rotate_180(opencv_image& visual) {
    cv::Mat* image = visual.get_image();
    cv::rotate(*image, *image, cv::ROTATE_180);
}

template<> void rotate_270(opencv_image& visual) {
    cv::Mat* image = visual.get_image();
    cv::rotate(*image, *image, cv::ROTATE_90_COUNTERCLOCKWISE);
}
#endif