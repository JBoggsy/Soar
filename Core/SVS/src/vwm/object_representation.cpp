#include "object_representation.h"
#include <math.h>
#include <algorithm>


//////////////////////////////////////////////////
//SECTION: `hand_crafted_object_representation` //
//////////////////////////////////////////////////
const cv::Scalar hand_crafted_object_representation::COLOR_RED(255, 0, 0, 255);
const cv::Scalar hand_crafted_object_representation::COLOR_GREEN(0, 255, 0, 255);
const cv::Scalar hand_crafted_object_representation::COLOR_BLUE(0, 0, 255, 255);
const cv::Scalar hand_crafted_object_representation::COLOR_CYAN(0, 255, 255, 255);
const cv::Scalar hand_crafted_object_representation::COLOR_MAGENTA(255, 0, 255, 255);
const cv::Scalar hand_crafted_object_representation::COLOR_YELLOW(255, 255, 0, 255);
const cv::Scalar hand_crafted_object_representation::COLOR_BLACK(15, 15, 15, 255);
const cv::Scalar hand_crafted_object_representation::COLOR_WHITE(240, 240, 240, 255);

const std::vector<cv::Scalar> hand_crafted_object_representation::COLORS{
    hand_crafted_object_representation::COLOR_RED,
    hand_crafted_object_representation::COLOR_GREEN,
    hand_crafted_object_representation::COLOR_BLUE,
    hand_crafted_object_representation::COLOR_CYAN,
    hand_crafted_object_representation::COLOR_MAGENTA,
    hand_crafted_object_representation::COLOR_YELLOW,
    hand_crafted_object_representation::COLOR_BLACK,
    hand_crafted_object_representation::COLOR_WHITE
};

const std::vector<std::string> hand_crafted_object_representation::COLOR_NAMES{
    "red",
    "green",
    "blue",
    "cyan",
    "magenta",
    "yellow",
    "black",
    "white"
};


hand_crafted_object_representation::hand_crafted_object_representation(int _border_size) {
    border_size = _border_size;
    shape = cv::Size2d(0, 0);
    diagonal_size = 0;
}

hand_crafted_object_representation::hand_crafted_object_representation(opencv_image* image, int _border_size) {
    // Initialize the basic image and mask data
    image->get_image()->copyTo(base_image);
    border_size = _border_size;
    cv::copyMakeBorder(base_image, base_image, _border_size, _border_size, _border_size, _border_size, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));
    cv::extractChannel(base_image, mask, 3);
    mask = mask > 0;
    shape = mask.size();
    diagonal_size = sqrt(pow(shape.width, 2) + pow(shape.height, 2));
}

void hand_crafted_object_representation::update_image(opencv_image* image) {
    // Update the basic image and mask data
    image->get_image()->copyTo(base_image);
    cv::copyMakeBorder(base_image, base_image, border_size, border_size, border_size, border_size, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));
    cv::extractChannel(base_image, mask, 3);
    mask = mask > 0;
    shape = mask.size();
    diagonal_size = sqrt(pow(shape.width, 2) + pow(shape.height, 2));

    // Reset the lazy-computed properties
    object_image_generated = false;
    contours_calculated = false;
    contour_calculated = false;
    mask_bbox_calculated = false;
    min_area_rect_calculated = false;
    contour_image_generated = false;
    ellipsity_calculated = false;
    line_segments_calculated = false;
    corners_calculated = false;
    corner_descriptors_calculated = false;
    moments_calculated = false;
}

void hand_crafted_object_representation::generate_object_image() {
    // Generate the object image
    cv::Mat mask_full_sized;
    cv::cvtColor(mask, mask_full_sized, cv::COLOR_GRAY2BGRA);
    mask_full_sized.convertTo(mask_full_sized, CV_32FC4, 1.0 / 255.0);
    base_image.convertTo(base_image, CV_32FC4, 1.0 / 255.0);
    object_image = cv::Mat::zeros(base_image.size(), CV_32FC4);
    object_image = base_image.mul(mask_full_sized);
    base_image.convertTo(base_image, CV_32FC4, 255.0);
    object_image.convertTo(object_image, CV_32FC4, 255.0);
    // Generate the grayscaled object image
    cv::cvtColor(object_image, object_image_gray, cv::COLOR_RGBA2GRAY);

    // Generate the object color
    cv::Scalar mean_color = cv::mean(object_image, mask);
    object_color = cv::Scalar(mean_color[0], mean_color[1], mean_color[2], mean_color[3]);
    #include <limits>

    double minDistance = std::numeric_limits<double>::max();
    int closestIndex = 0;
    for (size_t i = 0; i < COLORS.size(); i++) {
        cv::Scalar candidate = COLORS[i];
        double distance = sqrt(
            pow(object_color[0] - candidate[0], 2) +
            pow(object_color[1] - candidate[1], 2) +
            pow(object_color[2] - candidate[2], 2)
        );
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = static_cast<int>(i);
        }
    }
    object_color_name = COLOR_NAMES[closestIndex];
    object_image_generated = true;

}

void hand_crafted_object_representation::calculate_contours() {
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() == 0) {
        set_object_null(true);
        return;
    }
    contour = contours[0];

    if (all_contour_points.size() > 0) {
        all_contour_points.clear();
    }
    for (const auto& contour : contours) {
        all_contour_points.insert(all_contour_points.end(), contour.begin(), contour.end());
    }
    contours_calculated = true;
    contour_calculated = true;
}

void hand_crafted_object_representation::generate_contour_image() {
    if (!contours_calculated) calculate_contours();
    if (get_object_null()) {
        contour_image = cv::Mat::zeros(shape, CV_8UC3);
        return;
    }
    contour_image = cv::Mat::zeros(shape, CV_8UC3);
    cv::drawContours(contour_image, contours, -1, cv::Scalar(255, 255, 255), 1);
    contour_image_generated = true;
}

void hand_crafted_object_representation::calculate_mask_bbox() {
    if (!contours_calculated) calculate_contours();
    if (get_object_null()) {
        mask_bbox = cv::Rect2d(0, 0, 0, 0);
        return;
    }
    mask_bbox = cv::boundingRect(all_contour_points);
    mask_bbox_calculated = true;
}

void hand_crafted_object_representation::calculate_min_area_rect() {
    if (!contours_calculated) calculate_contours();
    if (get_object_null()) {
        min_area_rect = cv::RotatedRect(cv::Point2f(0, 0), cv::Size2f(0, 0), 0);
        return;
    }
    min_area_rect = cv::minAreaRect(all_contour_points);
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

    // Calculate the center of the corners
    cv::Point2f center(0, 0);
    for (cv::Vec2f corner : corners) {
        center.x += corner[0];
        center.y += corner[1];
    }
    center *= (1.0 / corners.size());

    // Sort corners in counterclockwise order
    std::sort(corners.begin(), corners.end(), [center](const cv::Point2f& a, const cv::Point2f& b) {
        double angle_a = atan2(a.y - center.y, a.x - center.x);
        double angle_b = atan2(b.y - center.y, b.x - center.x);
        return angle_a < angle_b;
    });

    // Convert corners to keypoints
    for (int i = 0; i < corners.size(); i++) {
        cv::KeyPoint keypoint = cv::KeyPoint(corners[i], 16.0);
        corner_keypoints.push_back(keypoint);
    }

    // Calculate corner angles
    cv::Vec2f corner_a;
    cv::Vec2f corner_b;
    cv::Vec2f corner_c;
    for (int i = 0; i < corners.size(); i++) {
        if (i == 0) {
            corner_a = corners.back();
        } else {
            corner_a = corners[i-1];
        }
        corner_b = corners[i];
        if (i == corners.size() - 1) {
            corner_c = corners[0];
        } else {
            corner_c = corners[i+1];
        }
        cv::Vec2f vector_a = corner_a - corner_b;
        cv::Vec2f vector_b = corner_c - corner_b;
        double dot_product = vector_a.dot(vector_b);
        double magnitude_a = sqrt(vector_a.dot(vector_a));
        double magnitude_b = sqrt(vector_b.dot(vector_b));
        double angle = acos(dot_product / (magnitude_a * magnitude_b));
        corner_angles.push_back(angle);
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

std::vector<std::pair<double, cv::Mat*>>* hand_crafted_object_representation::get_best_affine_transforms(hand_crafted_object_representation* other, int num_transforms) {
    std::priority_queue<std::pair<double, cv::Mat*>, std::vector<std::pair<double, cv::Mat*>>, std::greater<std::pair<double, cv::Mat*>>> affine_queue;
    std::vector<std::pair<double, cv::Mat*>>* affine_transforms = new std::vector<std::pair<double, cv::Mat*>>();

    std::unordered_set<double> scores_seen = std::unordered_set<double>();
    std::vector<cv::Vec2f> all_self_points = get_corners();
    std::vector<cv::Vec2f> all_other_points = other->get_corners();
    for (size_t si = 0; si < all_self_points.size(); si++) {
        for (size_t sj = 0; sj < all_self_points.size(); sj++) {
            if (si == sj) { continue; }
            for (size_t sk = 0; sk < all_self_points.size(); sk++) {
                if (si <= sk || sj == sk) { continue; }
                cv::Point2f self_prev = all_self_points[si];
                cv::Point2f self_curr = all_self_points[sj];
                cv::Point2f self_next = all_self_points[sk];
                if (cv::norm(self_prev - self_curr) < 5 || cv::norm(self_prev - self_next) < 5 || cv::norm(self_curr - self_next) < 5) {
                    continue;
                }
                std::vector<cv::Point2f> self_points = { self_prev, self_curr, self_next };

                for (size_t oi = 0; oi < all_other_points.size(); oi++) {
                    for (size_t oj = 0; oj < all_other_points.size(); oj++) {
                        if (oi == oj) { continue; }
                        for (size_t ok = 0; ok < all_other_points.size(); ok++) {
                            if (oi <= ok || oj == ok) { continue; }
                            cv::Point2f other_prev = all_other_points[oi];
                            cv::Point2f other_curr = all_other_points[oj];
                            cv::Point2f other_next = all_other_points[ok];
                            if (cv::norm(other_prev - other_curr) < 5 || cv::norm(other_prev - other_next) < 5 || cv::norm(other_curr - other_next) < 5) {
                                continue;
                            }
                            std::vector<cv::Point2f> other_points = { other_prev, other_curr, other_next };
                            cv::Mat affine = cv::getAffineTransform(self_points, other_points);

                            cv::Mat transformed_mask;
                            cv::warpAffine(mask, transformed_mask, affine, other->mask.size(), cv::INTER_NEAREST);

                            double target_area = static_cast<double>(cv::countNonZero(other->mask));

                            cv::Mat target_intersection;
                            cv::bitwise_and(transformed_mask, other->mask, target_intersection);
                            double intersection_area = static_cast<double>(cv::countNonZero(target_intersection));

                            cv::Mat target_union;
                            cv::bitwise_or(transformed_mask, other->mask, target_union);
                            double union_area = static_cast<double>(cv::countNonZero(target_union));

                            // double score = intersection_area / union_area;
                            double score = intersection_area / target_area;
                            if (scores_seen.find(score) != scores_seen.end()) {
                                continue;
                            } else {
                                scores_seen.insert(score);
                            }
                            affine_queue.push(std::make_pair(score, new cv::Mat(affine)));

                            ///////////
                            // DEBUG //
                            // draw matching colored dots on self and other object images
                            ///////////
                            cv::Mat self_debug, other_debug;
                            get_object_image().convertTo(self_debug, CV_8UC3, 255.0);
                            other->get_object_image().convertTo(other_debug, CV_8UC3, 255.0);

                            // Define three matching colors: red, green, blue.
                            cv::Scalar colors[3] = { cv::Scalar(0, 0, 255, 255), cv::Scalar(0, 255, 0, 255), cv::Scalar(255, 0, 0, 255)};

                            for (size_t k = 0; k < 3; k++) {
                                cv::circle(self_debug, self_points[k], 4, colors[k], -1);
                                cv::circle(other_debug, other_points[k], 4, colors[k], -1);
                            }

                            int rows = std::max(self_debug.rows, other_debug.rows);
                            int cols = self_debug.cols + other_debug.cols;
                            cv::Mat combined_debug(rows, cols, self_debug.type(), cv::Scalar(0, 0, 0));
                            cv::Mat leftROI = combined_debug(cv::Rect(0, 0, self_debug.cols, self_debug.rows));
                            self_debug.copyTo(leftROI);
                            cv::Mat rightROI = combined_debug(cv::Rect(self_debug.cols, 0, other_debug.cols, other_debug.rows));
                            other_debug.copyTo(rightROI);

                            // Draw colored lines between corresponding points in self and other debug images.
                            for (size_t k = 0; k < 3; k++) {
                                cv::Point2f pt1 = self_points[k];
                                cv::Point2f pt2 = other_points[k] + cv::Point2f((float) self_debug.cols, 0.0);
                                cv::line(combined_debug, pt1, pt2, colors[k], 2);
                            }

                            std::string filename = "affine_debug/affine_debug_" + std::to_string(si) + "-" + std::to_string(sj) + "-" + std::to_string(sk) + "_" + std::to_string(oi) + "-" + std::to_string(oj) + "-" + std::to_string(ok) + "_" + std::to_string(score) + ".png";
                            cv::imwrite(filename, combined_debug);

                            ///////////
                            // DEBUG //
                            ///////////

                            if (affine_queue.size() > num_transforms && num_transforms > 0) {
                                cv::Mat* worst_affine = affine_queue.top().second;
                                affine_queue.pop();
                                delete worst_affine;
                            }
                        }
                    }
                }
            }
        }
    }

    while (!affine_queue.empty()) {
        affine_transforms->push_back(affine_queue.top());
        affine_queue.pop();
    }
    return affine_transforms;
}

cv::Mat hand_crafted_object_representation::get_corner_affine_transform(hand_crafted_object_representation* other) {
    if (get_num_corners() != other->get_num_corners()) {
        throw std::invalid_argument("Both objects must have the same number of corners.");
    }

    std::vector<cv::Vec2f> self_corners = get_corners();
    std::vector<cv::Vec2f> other_corners = other->get_corners();
    return cv::estimateAffine2D(self_corners, other_corners);
}

float hand_crafted_object_representation::get_masks_iou(hand_crafted_object_representation* other) {
    cv::Mat intersection;
    cv::bitwise_and(mask, other->get_mask(), intersection);
    double intersection_area = static_cast<double>(cv::countNonZero(intersection));

    cv::Mat union_mask;
    cv::bitwise_or(mask, other->get_mask(), union_mask);
    double union_area = static_cast<double>(cv::countNonZero(union_mask));

    return static_cast<float>(intersection_area / union_area);
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

std::string hand_crafted_object_representation::to_string() {
    std::string out = "(" + std::to_string(shape.width) +
                      "," + std::to_string(shape.height) +
                      "," + object_color_name +
                      "," + std::to_string(get_min_rect_angle()) +
                      "," + std::to_string(get_min_rect_height()) +
                      "," + std::to_string(get_min_rect_width()) +
                      "," + std::to_string(get_ellipsity()) +
                      "," + std::to_string(get_num_corners()) +
                      "," + std::to_string(get_num_sides()) + ")";
    return out;
}

#ifdef ENABLE_TORCH
jepa_object_representation::jepa_object_representation() {
    shape = cv::Size2d(0, 0);
    diagonal_size = 0;
    border_size = 0;
}

jepa_object_representation::jepa_object_representation(opencv_image* image) {
    update_image(image);
}

jepa_object_representation::jepa_object_representation(opencv_image* image, img_factory_jepa* jepa_model) {
    update_jepa_model(jepa_model);
    update_image(image);
}

jepa_object_representation::jepa_object_representation(img_factory_jepa* jepa_model) {
    update_jepa_model(jepa_model);
}

jepa_object_representation::jepa_object_representation(img_factory_jepa* jepa_model, token_sequence* tokens) {
    update_jepa_model(jepa_model);
    update_tokens(tokens);
}

jepa_object_representation::~jepa_object_representation() {
    if (tokens != NULL) {
        delete tokens;
        tokens = NULL;
    }
}

void jepa_object_representation::update_image(opencv_image* image) {
    hand_crafted_object_representation::update_image(image);
    image_generated = true;
    if (model_loaded && !tokens_generated) {
        token_sequence* new_tokens = new token_sequence();
        jepa_model->encode(*image->get_image(), new_tokens);
        update_tokens(new_tokens);
    }
}

void jepa_object_representation::update_tokens(token_sequence* tokens) {
    if (this->tokens != NULL) {
        delete this->tokens;
    }
    this->tokens = tokens;

    if (token_summary != NULL) {
        delete token_summary;
    }
    token_summary = new cv::Mat();
    jepa_model->summarize(tokens, *token_summary);
    tokens_generated = true;

    if (model_loaded && !image_generated) {
        cv::Mat* new_image_mat = new cv::Mat();
        jepa_model->decode(tokens, *new_image_mat);
        opencv_image* new_image = new opencv_image();
        new_image->update_image(*new_image_mat);
        delete new_image_mat;
        update_image(new_image);
        delete new_image;
    }
}

void jepa_object_representation::update_jepa_model(img_factory_jepa* jepa_model) {
    this->jepa_model = jepa_model;
    model_loaded = true;

    if (!tokens_generated && image_generated) {
        token_sequence* new_tokens = new token_sequence();
        jepa_model->encode(base_image, new_tokens);
        update_tokens(new_tokens);
    }
    else if (tokens_generated && !image_generated) {
        cv::Mat* new_image_mat = new cv::Mat();
        jepa_model->decode(tokens, *new_image_mat);
        opencv_image* new_image = new opencv_image();
        new_image->update_image(*new_image_mat);
        delete new_image_mat;
        update_image(new_image);
        delete new_image;
    }
}

double jepa_object_representation::get_shape_distance(jepa_object_representation* other) {
    if (jepa_model == NULL || other->jepa_model == NULL) {
        throw std::runtime_error("JEPA model is not loaded.");
    }
    if (tokens == NULL || other->tokens == NULL) {
        throw std::runtime_error("Tokens are not generated.");
    }
    if (token_summary == NULL || other->token_summary == NULL) {
        throw std::runtime_error("Token summary is not generated.");
    }

    double dot_product = this->token_summary->dot(*other->token_summary);
    double this_norm = cv::norm(*token_summary);
    double other_norm = cv::norm(*other->token_summary);

    if (this_norm == 0 || other_norm == 0) {
        throw std::runtime_error("Cannot compute shape distance: one of the token summaries has zero norm.");
    }
    return (dot_product / (this_norm * other_norm));
}

#endif

//!SECTION
