#pragma once
#ifdef ENABLE_OPENCV

// Standard includes
#include <string>
#include <vector>
// Third-party includes
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
// SVS includes
#include "image.h"

class object_representation
{
public:
    virtual ~object_representation() {}
    static std::vector<object_representation*> get_object_representations(opencv_image* image);
};


class hand_crafted_object_representation : public object_representation
{
public:
    hand_crafted_object_representation(opencv_image* _base_image, cv::Mat* _mask);
    ~hand_crafted_object_representation() {}
    static cv::Mat get_edges(cv::Mat image, int low_thresh, float ratio, int ksize, int blur_ksize, bool invert);
    static std::pair<cv::Mat, cv::Mat> get_regions(cv::Mat edge_mask);
    static std::pair<cv::Mat, cv::Mat> get_watershed_markers(cv::Mat product_image, cv::Mat seed_regions, cv::Mat unknown_regions);
    static std::map<int, cv::Mat> get_watershed_object_masks(cv::Mat watershed_markers);
    static std::vector<hand_crafted_object_representation*> get_object_representations(opencv_image* image);
    static std::pair<std::map<int, cv::Mat>, std::vector<cv::Mat>> segment_product_image(cv::Mat product_img);
private:
    static const int MIN_CONTOUR_POINTS = 32;

    opencv_image base_image;
    cv::Mat mask;
    cv::Size2d shape;
    std::vector<std::vector<cv::Point>> contours;
    cv::Rect2d mask_bbox;
    cv::Mat object_image;
    cv::Mat object_image_gray;
    double diagonal_size;
    cv::Mat contour_image;
    cv::RotatedRect min_area_rect;
    std::vector<cv::Vec4f> line_segments;
    std::vector<cv::Vec2f> corners;
    std::vector<cv::KeyPoint> corner_keypoints;
    std::vector<std::vector<int>> corner_descriptors;
    cv::Moments moments;
    double hu_moments[7];
};
#endif