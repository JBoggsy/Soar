#include "object_representation.h"
#include <math.h>
#include <algorithm>

//////////////////////////////////////
//SECTION: `object_representation` //
//////////////////////////////////////
int get_object_representations(opencv_image* image, std::vector<object_representation*> &object_representations) {
    throw std::runtime_error("Not implemented");
}

//!SECTION


//////////////////////////////////////////////////
//SECTION: `hand_crafted_object_representation` //
//////////////////////////////////////////////////

int hand_crafted_object_representation::get_object_representations(opencv_image* image, std::vector<hand_crafted_object_representation*> &object_representations) {
    // Ensure the input image is valid
    if (image == NULL || image->get_image()->empty() || image->get_image()->cols <= 0 || image->get_image()->rows <= 0) {
        throw std::invalid_argument("Invalid input image");
    }

    // Create a bordered version of the image, since various computer vision
    // algorithms get confused by objects that are too close to the edge of the
    // image.
    int border_size = 64;
    cv::Mat bordered_image;
    cv::copyMakeBorder(*image->get_image(), bordered_image, border_size, border_size, border_size, border_size, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));

    // Segment the image into object masks
    std::vector<cv::Mat> masks;
    segment_image(bordered_image, masks);

    // Create object representations for each mask
    for (int i = 1; i < masks.size(); i++) {
        hand_crafted_object_representation* object_rep = new hand_crafted_object_representation(bordered_image, masks[i], border_size);
        object_representations.push_back(object_rep);
    }

    return masks.size();
}

int hand_crafted_object_representation::segment_image(cv::Mat image, std::vector<cv::Mat> &masks) {
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

hand_crafted_object_representation::hand_crafted_object_representation(cv::Mat _base_image, cv::Mat _mask, int _border_size) {
    // Initialize the basic image and mask data
    _base_image.copyTo(base_image);
    _mask.copyTo(mask);
    border_size = _border_size;
    shape = mask.size();
    diagonal_size = sqrt(pow(shape.width, 2) + pow(shape.height, 2));
}

void hand_crafted_object_representation::generate_object_image() {
    cv::Mat mask_full_sized;
    cv::cvtColor(mask, mask_full_sized, cv::COLOR_GRAY2BGRA);
    mask_full_sized.convertTo(mask_full_sized, CV_32FC4, 1.0 / 255.0);
    object_image = cv::Mat::zeros(base_image.size(), CV_32FC4);
    object_image = base_image.mul(mask_full_sized);
    cv::cvtColor(object_image, object_image_gray, cv::COLOR_RGBA2GRAY);
    object_image_generated = true;
}

void hand_crafted_object_representation::calculate_contours() {
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    contour = contours[0];
    contours_calculated = true;
    contour_calculated = true;
}

void hand_crafted_object_representation::generate_contour_image() {
    if (!contours_calculated) calculate_contours();
    contour_image = cv::Mat::zeros(shape, CV_8UC3);
    cv::drawContours(contour_image, contours, 0, cv::Scalar(255, 255, 255), 1);
    contour_image_generated = true;
}

void hand_crafted_object_representation::calculate_mask_bbox() {
    if (!contours_calculated) calculate_contours();
    mask_bbox = cv::boundingRect(contours[0]);
    mask_bbox_calculated = true;
}

void hand_crafted_object_representation::calculate_min_area_rect() {
    if (!contours_calculated) calculate_contours();
    min_area_rect = cv::minAreaRect(contours[0]);
    min_area_rect_calculated = true;
}

void hand_crafted_object_representation::calculate_ellipsity() {
    if (!min_area_rect_calculated) calculate_min_area_rect();
    cv::Point2f elliptic_center = min_area_rect.center;
    cv::Size2f elliptic_size = min_area_rect.size;
    double elliptic_theta = min_area_rect.angle;

    double h = elliptic_center.x;
    double k = elliptic_center.y;

    double rect_width = elliptic_size.width / 2;
    double rect_height = elliptic_size.height / 2;

    if (rect_height > rect_height) {
        elliptic_theta = elliptic_theta + 90;
    }
    elliptic_theta = elliptic_theta * 3.14159265 / 180;

    double semimajor_axis = std::max(rect_width, rect_height);
    double semiminor_axis = std::min(rect_width, rect_height);

    double elliptic_points = 0.0;
    for (int i = 0; i < contours[0].size(); i++) {
        cv::Point contour_point = contours[0][i];
        double distance = (pow((contour_point.x - h) * cos(elliptic_theta) +
                               (contour_point.y - k) * sin(elliptic_theta), 2) / pow(semimajor_axis, 2)) +
                          (pow((contour_point.x - h) * sin(elliptic_theta) -
                               (contour_point.y - k) * cos(elliptic_theta), 2) / pow(semiminor_axis, 2));
        if ((1-distance) < 0.025) {
            elliptic_points += 1;
        }
    }
    ellipsity = elliptic_points / contours[0].size();
    ellipsity_calculated = true;
}

void hand_crafted_object_representation::calculate_line_segments() {
    int length_threshold = 10;
    float distance_threshold = 1.41421356f;
    double canny_th1 = 1.0;
    double canny_th2 = 255.0;
    int canny_aperture_size = 0;
    bool do_merge = true;

    cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(
        length_threshold,
        distance_threshold,
        canny_th1,
        canny_th2,
        canny_aperture_size,
        do_merge
    );
    cv::Mat contour_image_copy = get_contour_image().clone();
    cv::cvtColor(contour_image_copy, contour_image_copy, cv::COLOR_BGR2GRAY);
    fld->detect(contour_image_copy, line_segments);
}

void hand_crafted_object_representation::calculate_corners() {
    if (!line_segments_calculated) calculate_line_segments();
    int max_corners = line_segments.size();
    double quality_level = 0.05;
    double min_distance = diagonal_size / 100;
    cv::InputArray mask_input = cv::noArray();
    int block_size = 3;
    bool use_harris_detector = true;
    double k = 0.04;
    cv::goodFeaturesToTrack(
        mask,
        corners,
        max_corners,
        quality_level,
        min_distance,
        mask_input,
        block_size,
        use_harris_detector,
        k
    );
    for (int i = 0; i < corners.size(); i++) {
        cv::KeyPoint keypoint = cv::KeyPoint(corners[i], 16.0);
        corner_keypoints.push_back(keypoint);
    }
    corners_calculated = true;
}

void hand_crafted_object_representation::calculate_corner_descriptors() {
    if (!corners_calculated) calculate_corners();
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->compute(object_image, corner_keypoints, corner_descriptors);
    corner_descriptors_calculated = true;
}

void hand_crafted_object_representation::calculate_moments() {
    moments = cv::moments(contours[0]);
    cv::HuMoments(moments, hu_moments);
    moments_calculated = true;
}

double hand_crafted_object_representation::get_shape_distance(hand_crafted_object_representation* other) {
    return cv::matchShapes(this->get_contours()[0], other->get_contours()[0], cv::CONTOURS_MATCH_I2, 0.0);
}

void hand_crafted_object_representation::_subdivide_contours() {
    std::vector<cv::Point> new_contour = std::vector<cv::Point>();
    cv::Point point_a = contours[0][contours[0].size() - 1];
    std::vector<cv::Point>::iterator point_b_itr = contours[0].begin();
    for (point_b_itr; point_b_itr != contours[0].end(); point_b_itr++) {
        cv::Point point_b = *point_b_itr;
        cv::Point new_point = cv::Point((point_a.x + point_b.x) / 2, (point_a.y + point_b.y) / 2);
        new_contour.push_back(point_a);
        new_contour.push_back(new_point);
        point_a = point_b;
    }
    contours[0] = new_contour;
}

//!SECTION
