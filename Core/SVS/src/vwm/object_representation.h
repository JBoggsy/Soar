#pragma once
#ifdef ENABLE_OPENCV

// Standard includes
#include <string>
#include <vector>
#include <queue>
// Third-party includes
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
// SVS includes
#include "image.h"

class object_representation
{
public:
    virtual ~object_representation() {}
    virtual cv::Mat get_mask() = 0;
    virtual cv::Rect2d get_mask_bbox() = 0;
};


class hand_crafted_object_representation : public object_representation
{
public:
    hand_crafted_object_representation(opencv_image* image, int border_size=16);
    ~hand_crafted_object_representation() {}

    /**
     * @brief Segments the given product image into object masks via color-based
     * segmentation.
     *
     * @param product_img The product image to segment.
     * @param masks The resulting object masks.
     *
     * @return The number of object masks extracted
     */
    static int segment_image(cv::Mat image, std::vector<cv::Mat> &masks);

    cv::Mat get_base_image() { return base_image; }
    cv::Mat get_mask() { return mask; }
    cv::Vec4i get_border_size() { return border_size; }
    cv::Size2d get_shape() { return shape; }
    double get_diagonal_size() { return diagonal_size; }

    cv::Mat get_object_image() {if (!object_image_generated) generate_object_image(); return object_image;}
    cv::Mat get_object_image_gray() {if (!object_image_generated) generate_object_image(); return object_image_gray;}

    std::vector<std::vector<cv::Point>> get_contours() {if (!contours_calculated) calculate_contours(); return contours;}
    std::vector<cv::Point> get_contour() {if (!contour_calculated) calculate_contours(); return contour;}
    cv::Mat get_contour_image() {if (!contour_image_generated) generate_contour_image(); return contour_image;}
    cv::Rect2d get_mask_bbox() {if (!mask_bbox_calculated) calculate_mask_bbox(); return mask_bbox;}
    cv::RotatedRect get_min_area_rect() {if (!min_area_rect_calculated) calculate_min_area_rect(); return min_area_rect;}
    double get_ellipsity() { return ellipsity; }

    std::vector<cv::Vec4f> get_line_segments() {if (!line_segments_calculated) calculate_line_segments(); return line_segments;}
    int get_num_sides() {if (!line_segments_calculated) calculate_line_segments(); return line_segments.size();}
    std::vector<cv::Vec2f> get_corners() {if (!corners_calculated) calculate_corners(); return corners;}
    int get_num_corners() {if (!corners_calculated) calculate_corners(); return corners.size();}
    std::vector<double> get_corner_angles() {if (!corners_calculated) calculate_corners(); return corner_angles;}
    std::vector<cv::KeyPoint> get_corner_keypoints() {if (!corners_calculated) calculate_corners(); return corner_keypoints;}
    std::vector<std::vector<int>> get_corner_descriptors() {if (!corner_descriptors_calculated) calculate_corner_descriptors(); return corner_descriptors;}

    cv::Moments get_moments() {if (!moments_calculated) calculate_moments(); return moments;}
    double* get_hu_moments() {if (!moments_calculated) calculate_moments(); return hu_moments;}

    /**
     * @brief Uses the `cv::matchShapes` method to compute how similar the shape
     * of this object is to the shape of another object.
     *
     * The `cv::matchShapes` method computes a similarity metric between two
     * shapes by comparing their contours. The metric is based on the Hu moments
     * of the two shapes, which are invariant to translation, rotation, and scale.
     *
     * @param other The other object to compare to.
     */
    double get_shape_distance(hand_crafted_object_representation* other);

    /**
     * @brief Computes the best affine transformations to align this object with
     * another object.
     *
     * This method uses the `cv::getAffineTransform` method to compute the best
     * affine transformation to align this object with another object.
     *
     * @param other The other object to align with.
     * @param num_transforms The number of best affine transformations to compute.
     */
    std::vector<cv::Mat*> get_best_affine_transforms(hand_crafted_object_representation* other, int num_transforms);

private:
    static const int MIN_CONTOUR_POINTS = 32;

    // Basic image and mask data, computed in constructor
    cv::Mat base_image;
    cv::Mat mask;
    int border_size;
    cv::Size2d shape;
    double diagonal_size;

    // Object image, lazy-computed
    cv::Mat object_image;
    cv::Mat object_image_gray;
    bool object_image_generated = false;

    // Contours and related, lazy-computed
    std::vector<std::vector<cv::Point>> contours;
    bool contours_calculated = false;
    std::vector<cv::Point> contour;
    bool contour_calculated = false;
    cv::Rect2d mask_bbox;
    bool mask_bbox_calculated = false;
    cv::RotatedRect min_area_rect;
    bool min_area_rect_calculated = false;
    cv::Mat contour_image;
    bool contour_image_generated = false;
    double ellipsity;
    bool ellipsity_calculated = false;

    // Line segments, corners, and their descriptors, lazy-computed
    std::vector<cv::Vec4f> line_segments;
    bool line_segments_calculated = false;
    std::vector<cv::Vec2f> corners;
    std::vector<double> corner_angles;
    std::vector<cv::KeyPoint> corner_keypoints;
    bool corners_calculated = false;
    std::vector<std::vector<int>> corner_descriptors;
    bool corner_descriptors_calculated = false;

    // Moments and Hu moments, lazy-computed
    cv::Moments moments;
    double hu_moments[7];
    bool moments_calculated = false;

    void generate_object_image();

    void calculate_contours();
    void generate_contour_image();
    void calculate_mask_bbox();
    void calculate_min_area_rect();
    void calculate_ellipsity();

    void calculate_line_segments();
    void calculate_corners();
    void calculate_corner_descriptors();

    void calculate_moments();

    /**
     * @brief Subdivides the contours of the object mask.
     *
     * This helper function subdivides the contours of the object mask into
     * smaller segments by adding new points halfway between each pair of points
     * in the contour. This is useful for "adding resolution" to the contours,
     * and is necessary for using shape context distance metrics for certain
     * simple shapes.
    */
    void _subdivide_contours();
};


#define OBJ_REP_TYPE hand_crafted_object_representation
#endif
