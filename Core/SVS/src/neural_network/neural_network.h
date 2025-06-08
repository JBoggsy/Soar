#pragma once
#ifdef ENABLE_TORCH
// standard includes
#include <string>
// third-party includes
#include <opencv2/opencv.hpp>
// SVS includes
// #include "image.h"
#include "latent_representation.h"
#include "token_sequence.h"

// forward declarations
class torch_module_wrapper;
class vae_base_model_wrapper;
class vae_vcd_model_wrapper;
class img_factory_jepa_wrapper;

/**
 * @brief An abstract class for interfacing with a PyTorch-based neural network.
 *
 * This class is a base class for classes which wrap around PyTorch neural
 * networks that have been traced to TorchScript. Concrete sub-classes should be
 * very simple to implement, because all they need to do is have their methods
 * call the corresponding methods from the `torch_module_wrapper` class they
 * wrap. This class is necessary because the PyTorch C++ library defines a
 * `Symbol` class which conflicts with the `Symbol` class defined in the Soar
 * codebase. The `torch_module_wrapper` class allows us to avoid including the
 * PyTorch headers in the Soar codebase, but since the `.h` file for that class
 * requires the PyTorch headers, we need to define this class and its subclasses
 * in a separate file (and thus as a separate class).
 */
class neural_network
{
private:
    /**
     * @brief The wrapper object for the PyTorch module underlying this model.
     *
     * The `neural_network` should just be a direct wrapper around this, meaning
     * it should have all the same methods as the `torch_module_wrapper`
     * subclass it wraps, and just call the corresponding methods directly.
     */
    torch_module_wrapper* module;
    bool module_loaded = false;
public:
    neural_network();
    neural_network(std::string traced_script_path);
    ~neural_network();
    void load_traced_script(std::string traced_script_path);
    cv::Mat forward(cv::Mat& input);

    bool get_module_loaded() { return module_loaded; }
};


class vae_base_model : public neural_network
{
private:
    vae_base_model_wrapper* module;
    bool module_loaded = false;

public:
    vae_base_model();
    vae_base_model(std::string traced_script_path);
    ~vae_base_model();
    void load_traced_script(std::string traced_script_path);

    void encode(cv::Mat& input, latent_representation* latent);
    void decode(latent_representation* latent, cv::Mat& output);
    bool get_module_loaded() { return module_loaded; }
};


class vae_vcd_model : public neural_network
{
private:
    vae_vcd_model_wrapper* module;
    bool module_loaded = false;
public:
    vae_vcd_model();
    vae_vcd_model(std::string traced_script_path);
    ~vae_vcd_model();
    void load_traced_script(std::string traced_script_path);

    /**
     * @brief "Encodes" a single-element probability latent into a latent
     * representation in the base VAE's latent space. Used for generation.
     */
    void encode(latent_representation* input, latent_representation* latent);

    /**
     * @brief "Decodes" a latent representation in the base VAE's latent space
     * into a single-element probability latent. Used for recognition.
     */
    void decode(latent_representation* latent, latent_representation* output);
    bool get_module_loaded() { return module_loaded; }
};


class img_factory_jepa : public neural_network
{
private:
    img_factory_jepa_wrapper* module;
    bool module_loaded = false;

public:
    img_factory_jepa();
    img_factory_jepa(std::string traced_script_path);
    ~img_factory_jepa();

    void load_traced_script(std::string traced_script_path);
    bool get_module_loaded() { return module_loaded; }

    /**
     * @brief Encodes an image into a token sequence.
     *
     * @param input The input image as a cv::Mat.
     * @param tokens The output token sequence.
     */
    void encode(cv::Mat& input, token_sequence* tokens);

    /**
     * @brief Summarizes a token sequence into a single vector.
     *
     * Cosine similarity between two summarized token sequences can be used to
     * determine how similar the two token sequences are.
     *
     * @param tokens The input token sequence.
     * @param output The output summarized vector as a cv::Mat.
     */
    void summarize(token_sequence* tokens, cv::Mat& output);

    /**
     * @brief Decodes a token sequence into an image.
     *
     * @param tokens The input token sequence.
     * @param output The output image as a cv::Mat.
     */
    void decode(token_sequence* tokens, cv::Mat& output);

    /**
     * @brief Deobscures a token sequence using a conditioning sequence.
     *
     * Given the unobscured original image of an object and a
     * possibly-transformed, possibly-obscured final image of the object,
     * generate the unobscured final image.
     *
     * @param source The original token sequence representing the unobscured
     * source image.
     * @param conditioning The token sequence representing the possibly-obscured
     * and possibly-transformed final image.
     * @param output The output token sequence representing the deobscured
     * final image.
     */
    void deobscure(token_sequence* source, token_sequence* conditioning, token_sequence* output);

    /**
     * @brief Extracts a possibly-transformed, possibly-obscured segment
     * corresponding to the source token sequence from the base token sequence.
     *
     * Given the unobscured original image of an object and a base image which
     * contains a possibly-transformed, possibly-obscured instance of the
     * original object as well as one or more other objects, generate a token
     * sequence representing specifically the visible portion of the
     * possibly-transformed, possibly-obscured instance of the original object.
     *
     * @param source The original token sequence representing the unobscured
     * source image.
     * @param base The base token sequence representing the image containing
     * the possibly-transformed, possibly-obscured instance of the original
     * object.
     * @param output The output token sequence representing the extracted
     * possibly-transformed, possibly-obscured instance of the original object.
     */
    void extract(token_sequence* source, token_sequence* base, token_sequence* output);

    /**
     * @brief Returns a distance measure between two token sequences.
     */
    double get_shape_distance(token_sequence* tokens1, token_sequence* tokens2);

};
#endif
