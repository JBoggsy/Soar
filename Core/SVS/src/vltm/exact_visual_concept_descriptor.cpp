// C++ std libraries
#include <exception>
// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
// SVS includes
#include "visual_concept_descriptor.h"
#include "exact_visual_concept_descriptor.h"
#include "image.h"


template<typename img_t>
exact_visual_concept_descriptor<img_t>::exact_visual_concept_descriptor(std::string entity_id) {
    _entity_id = entity_id;
    archetype = img_t();
}

template<typename img_t>
void exact_visual_concept_descriptor<img_t>::store_percept(img_t example) {
    archetype.copy_from(&example);
}

template<typename img_t>
double exact_visual_concept_descriptor<img_t>::recognize(img_t percept) {
    if (percept == archetype) { return 1.0; }
    else { return 0.0; }
}

template<typename img_t>
void exact_visual_concept_descriptor<img_t>::generate(img_t* output) {
    output->copy_from(&archetype);
}


#ifdef ENABLE_OPENCV
template<>
exact_visual_concept_descriptor<opencv_image>::exact_visual_concept_descriptor(std::string entity_id) {
    _entity_id = entity_id;
    archetype = opencv_image();
}

template<>
void exact_visual_concept_descriptor<opencv_image>::store_percept(opencv_image example) {
    archetype.copy_from(&example);
}

template<>
double exact_visual_concept_descriptor<opencv_image>::recognize(opencv_image percept) {
    int result_cols = percept.get_width() - archetype.get_width() + 1;
    int result_rows = percept.get_height() - archetype.get_height() + 1;

    cv::Mat result;
    result.create(result_rows, result_cols, CV_32FC1);

    cv::matchTemplate(*(percept.get_image()), *(archetype.get_image()), result, cv::TM_CCOEFF);
    // cv::imwrite("/home/boggsj/Coding/research/svs_experiments/recognize_test.png", result);

    double min, max;
    cv::Point minloc;
    cv::Point maxloc;

    cv::minMaxLoc(result, &min, &max, &minloc, &maxloc);

    return max;
}

template<>
void exact_visual_concept_descriptor<opencv_image>::generate(opencv_image* output) {
    output->copy_from(&archetype);
}


#endif

// Explicit instantiations of other EVCDs
template class exact_visual_concept_descriptor<basic_image>;
#ifdef ENABLE_ROS
template class exact_visual_archetype<pcl_image>;
#endif
