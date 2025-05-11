#ifdef ENABLE_TORCH
#include "jepa_visual_concept_descriptor.h"


template<typename img_t>
jepa_visual_concept_descriptor<img_t>::jepa_visual_concept_descriptor(std::string entity_id) {
    _entity_id = entity_id;
    _example = new img_t();
}

template<typename img_t>
jepa_visual_concept_descriptor<img_t>::~jepa_visual_concept_descriptor() {
}

template<typename img_t>
void jepa_visual_concept_descriptor<img_t>::store_percept(img_t example) {
    _example->copy_from(&example);
}

template<>
double jepa_visual_concept_descriptor<token_sequence>::recognize(token_sequence percept) {
    // Get token matrices from both sequences
    cv::Mat& example_tokens = _example->get_tokens();
    cv::Mat& percept_tokens = percept.get_tokens();

    // Verify dimensions match
    if (example_tokens.rows != percept_tokens.rows ||
        example_tokens.cols != percept_tokens.cols) {
        // Return maximum distance (lowest similarity) if dimensions don't match
        return std::numeric_limits<double>::max();
    }

    // Compute L2 distance between token matrices
    // Using OpenCV's norm function with NORM_L2 parameter
    double distance = cv::norm(example_tokens, percept_tokens, cv::NORM_L2);

    // Return the computed distance
    // Lower values indicate higher similarity
    return 1/distance;
}

template<>
void jepa_visual_concept_descriptor<token_sequence>::generate(token_sequence* output) {
    output->set_tokens(_example->get_tokens());
}

template class jepa_visual_concept_descriptor<token_sequence>;
#endif
