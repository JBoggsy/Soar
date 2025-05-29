#ifdef ENABLE_TORCH
#include "token_sequence.h"

void token_sequence::set_tokens(cv::Mat& tokens) {
    if (tokens.rows != _num_tokens || tokens.cols != _num_features) {
        set_num_features(tokens.cols);
        set_num_tokens(tokens.rows);
        _tokens = cv::Mat(_num_tokens, _num_features, CV_32F);
        _tokens = tokens.clone();
    }
    else {
        _tokens = tokens.clone();
    }
    _tokens.convertTo(_tokens, CV_32F);
}

void token_sequence::copy_from(token_sequence* other) {
    _num_tokens = other->_num_tokens;
    _num_features = other->_num_features;
    _is_padded = other->_is_padded;
    _padding_t = other->_padding_t;
    _padding_b = other->_padding_b;
    _padding_l = other->_padding_l;
    _padding_r = other->_padding_r;
    _is_scaled = other->_is_scaled;
    _scale_width = other->_scale_width;
    _scale_height = other->_scale_height;
    _tokens = other->_tokens.clone();
}
#endif
