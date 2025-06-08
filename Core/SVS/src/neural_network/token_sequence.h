#pragma once
#ifdef ENABLE_TORCH
// c++ includes
#include <vector>
// opencv includes
#include <opencv2/opencv.hpp>

/**
 * @brief Represents a sequence of tokens.
 *
 * This class is used to store a sequence of tokens, which can be used in
 * various CV or NLP tasks. This class stores the tokens as an OpenCV matrix as
 * a workaround for being unable to directly use at::Tensor in the Soar
 * codebase. Specifically, the tokens are stored as a 2D, single channel
 * (CV_32F) matrix in which each row represents a token and each column
 * represents a feature of the token.
 *
 * @note The name of this class is a a slight misnomer, as it does not contain a
 * tokenizer.
 */
class token_sequence
{
private:
    cv::Mat _tokens; // 2D matrix of tokens
    int _num_tokens; // number of tokens in the sequence (rows in the matrix)
    int _num_features; // number of features per token (columns in the matrix)

    // If the encoded image was padded, these values are used to
    // unpad the image after decoding.
    bool _is_padded; // whether the tokens are padded
    int _padding_t;  // padding for each side
    int _padding_b;
    int _padding_l;
    int _padding_r;

    // If the encoded image was downscaled, these values are used to
    // upscale the image after decoding.
    bool _is_scaled;
    float _scale_width; // scaling factor for width
    float _scale_height; // scaling factor for height

public:
    /**
     * @brief Construct a new token sequence object.
     *
     * @param num_tokens The number of tokens in the sequence.
     * @param num_features The number of features per token.
     */
    token_sequence()
        : _num_tokens(0), _num_features(0),
          _is_padded(false), _padding_t(0), _padding_b(0),
          _padding_l(0), _padding_r(0), _is_scaled(false),
          _scale_width(1.0f), _scale_height(1.0f) {}
    ~token_sequence() {}
    void copy_from(token_sequence* other);

    void set_padding(int t, int b, int l, int r) {
        _is_padded = true;
        _padding_t = t;
        _padding_b = b;
        _padding_l = l;
        _padding_r = r;
    }

    void set_scaling(float width, float height) {
        _is_scaled = true;
        _scale_width = width;
        _scale_height = height;
    }

    cv::Mat& get_tokens() { return _tokens; }
    void set_tokens(cv::Mat& tokens);

    void set_num_tokens(int num_tokens) { _num_tokens = num_tokens; }
    void set_num_features(int num_features) { _num_features = num_features; }
    int get_num_tokens() const { return _num_tokens; }
    int get_feature_dim() const { return _num_features; }
};

#endif
