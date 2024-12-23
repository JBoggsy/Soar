#include "object_representation.h"


std::vector<hand_crafted_object_representation*> hand_crafted_object_representation::get_object_representations(opencv_image* image) {
    std::vector<hand_crafted_object_representation*> object_representations = std::vector<hand_crafted_object_representation*>();
    cv::Mat image_mat = *image->get_image();
    auto [obj_masks, obj_layers] = segment_product_image(image_mat);
    std::map<int, cv::Mat>::iterator obj_mask_itr = obj_masks.begin();
    for (obj_mask_itr; obj_mask_itr != obj_masks.end(); obj_mask_itr++) {
        if (obj_mask_itr->first == 1) continue;
        hand_crafted_object_representation* object_representation = new hand_crafted_object_representation(image, &obj_mask_itr->second);
        object_representations.push_back(object_representation);
    }

    return object_representations;
}

cv::Mat hand_crafted_object_representation::get_edges(cv::Mat image, int low_thresh, float ratio, int ksize, int blur_ksize, bool invert) {
    // Ensure the input image is valid
    if (image.empty() || image.cols <= 0 || image.rows <= 0) {
        throw std::invalid_argument("Invalid input image");
    }

    // Ensure blur_ksize is positive and odd
    if (blur_ksize <= 0) {
        blur_ksize = 3; // Default to 3 if invalid
    } else if (blur_ksize % 2 == 0) {
        blur_ksize += 1; // Make it odd if even
    }

    cv::Mat blurred_img;
    cv::blur(image, blurred_img, cv::Size(blur_ksize, blur_ksize), cv::Point(-1, -1));
    cv::Mat detected_edges;
    cv::Canny(blurred_img, detected_edges, low_thresh, low_thresh * ratio, ksize);
    cv::Mat edge_mask;
    if (invert) {
        edge_mask = detected_edges == 0;
    } else {
        edge_mask = detected_edges != 0;
    }
    edge_mask = edge_mask.mul(image != 0);
    return edge_mask;
}

std::pair<cv::Mat, cv::Mat> hand_crafted_object_representation::get_regions(cv::Mat edge_mask) {
    cv::Mat dists_to_edge;
    cv::distanceTransform(edge_mask, dists_to_edge, cv::DIST_L2, 5);
    double minVal, maxVal;
    cv::minMaxLoc(dists_to_edge, &minVal, &maxVal);
    cv::Mat seed_regions;
    cv::threshold(dists_to_edge, seed_regions, 0.05 * maxVal, 255, 0);
    cv::Mat sure_bg;
    cv::dilate(edge_mask, sure_bg, cv::Mat::ones(3, 3, CV_8U), cv::Point(-1, -1), 1);
    cv::Mat unknown_regions;
    cv::subtract(sure_bg, seed_regions, unknown_regions);
    cv::dilate(unknown_regions, unknown_regions, cv::Mat::ones(3, 3, CV_8U), cv::Point(-1, -1), 1);
    return std::make_pair(seed_regions, unknown_regions);
}

std::pair<cv::Mat, cv::Mat> hand_crafted_object_representation::get_watershed_markers(cv::Mat product_image, cv::Mat seed_regions, cv::Mat unknown_regions) {
    cv::Mat marker_seeds;
    cv::connectedComponents(seed_regions, marker_seeds);
    marker_seeds = marker_seeds + 1;
    marker_seeds.setTo(0, unknown_regions > 0);

    cv::Mat flat_prod_img;
    cv::cvtColor(product_image, flat_prod_img, cv::COLOR_RGBA2RGB);
    cv::Mat watershed_markers;
    cv::watershed(flat_prod_img, marker_seeds);
    return std::make_pair(marker_seeds, watershed_markers);
}

std::map<int, cv::Mat> hand_crafted_object_representation::get_watershed_object_masks(cv::Mat watershed_markers) {
    std::map<int, cv::Mat> obj_masks;
    for (int obj_id = 0; obj_id < watershed_markers.rows; ++obj_id) {
        if (obj_id == -1) continue;
        cv::Mat mask = (watershed_markers == obj_id);
        obj_masks[obj_id] = mask;
    }
    return obj_masks;
}

std::pair<std::map<int, cv::Mat>, std::vector<cv::Mat>> hand_crafted_object_representation::segment_product_image(cv::Mat product_img) {
    cv::Mat edge_mask = get_edges(product_img, 75, 4.0f, 3, 3, true);
    auto [seed_regions, unknown_regions] = get_regions(edge_mask);
    auto [marker_seeds, watershed_markers] = get_watershed_markers(product_img, seed_regions, unknown_regions);
    std::map<int, cv::Mat> obj_masks = get_watershed_object_masks(watershed_markers);
    std::vector<cv::Mat> img_layers;
    for (const auto& [obj_id, obj_mask] : obj_masks) {
        cv::Mat img_layer;
        product_img.copyTo(img_layer, obj_mask);
        img_layers.push_back(img_layer);
    }
    return std::make_pair(obj_masks, img_layers);
}

hand_crafted_object_representation::hand_crafted_object_representation(opencv_image* _base_image, cv::Mat* _mask) {
    // Initialize the basic image and mask data
    base_image.copy_from(_base_image);
    _mask->copyTo(mask);
    object_image = mask * (*base_image.get_image());
    cv::cvtColor(object_image, object_image_gray, cv::COLOR_RGBA2GRAY);
    shape = mask.size();
    mask_bbox = cv::boundingRect(contours[0]);
    diagonal_size = sqrt(pow(shape.width, 2) + pow(shape.height, 2));

    // Calculate the contours of the mask
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    while (contours.size() < MIN_CONTOUR_POINTS) {
        std::vector<std::vector<cv::Point>> new_contours = std::vector<std::vector<cv::Point>>();
        cv::Point point_a = contours[0][contours[0].size() - 1];
        std::vector<cv::Point>::iterator point_b_itr = contours[0].begin();
        for (point_b_itr; point_b_itr != contours[0].end(); point_b_itr++) {
            cv::Point point_b = *point_b_itr;
            cv::Point new_point = cv::Point((point_a.x + point_b.x) / 2, (point_a.y + point_b.y) / 2);
            new_contours.push_back(std::vector<cv::Point>{point_a, new_point});
            point_a = point_b;
        }

    }
    contour_image = cv::Mat::zeros(shape, CV_8UC3);
    cv::drawContours(contour_image, contours, 0, cv::Scalar(255, 255, 255), 1);
    min_area_rect = cv::minAreaRect(contours[0]);

    // Calculate the line segments of the mask
    int length_threshold = 10;
    float distance_threshold = 1.41421356f;
    double canny_th1 = 50.0;
    double canny_th2 = 50.0;
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
    fld->detect(mask, line_segments);

    // Calculate the track points of the mask, as well as their ORB keypoints
    // and descriptors
    int max_corners = line_segments.size();
    double quality_level = 0.05;
    double min_distance = diagonal_size / 10;
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
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->compute(object_image_gray, corner_keypoints, corner_descriptors);

    // Calculate the moments of the mask
    moments = cv::moments(contours[0]);
    cv::HuMoments(moments, hu_moments);
}