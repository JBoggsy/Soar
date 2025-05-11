#ifdef ENABLE_TORCH
#include "token_sequence.h"

token_sequence::token_sequence(int num_tokens, int num_features) {
    _num_tokens = num_tokens;
    _num_features = num_features;
    _tokens = cv::Mat(num_tokens, num_features, CV_32F);
}

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
    _tokens = other->_tokens.clone();
}
#endif
