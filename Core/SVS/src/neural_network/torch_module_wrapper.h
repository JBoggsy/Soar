#pragma once
// standard includes
#include <string>
// third-party includes
#include <opencv2/opencv.hpp>
#include <torch/script.h>
// SVS includes
#include "latent_representation.h"


/**
 * @brief An abstract class for wrapping PyTorch modules via TorchScript.
 *
 * This is a base class from which model-specific subclasses can be derived to
 * wrap specific PyTorch models (or kinds of models). The base class only
 * provides a couple basic methods: a constructor that loads a traced script
 * from a file, and a `forward` method that takes an input tensor and returns
 * the output tensor from the model. Subclasses should also include any other
 * methods the model requires/offers (e.g., `encode`/`decode` for an
 * autoencoder).
 *
 * This class is necessary because the PyTorch C++ library defines a `Symbol`
 * class which conflicts with the `Symbol` class defined in the Soar codebase.
 * The `torch_module_wrapper` class allows us to avoid including the PyTorch
 * headers in the Soar codebase by using them here and in
 * `torch_module_wrapper.cpp` and then forward declaring the classes in
 * `neural_network.h`.
 */
class torch_module_wrapper
{
protected:
    /**
     * @brief Convert an opencv Mat to a PyTorch tensor.
     *
     * This method ensures the output tensor is a float tensor with values
     * normalized between 0 and 1. It also assumes the input Mat is an image,
     * because it permutes the dimensions to match the expected input format for a
     * PyTorch model (channel x height x width).
     *
     * @param input The input opencv Mat.
     * @param output The output PyTorch tensor.
     */
    void mat_to_tensor(cv::Mat& input, at::Tensor& output);

    /**
     * @brief Convert a PyTorch tensor to an opencv Mat.
     *
     * This method assumes the input tensor is an image, because it permutes the
     * dimensions to match the expected format for an opencv image (height x width x
     * channe`l).
     *
     * @param input The input PyTorch tensor.
     * @param output The output opencv Mat.
     */
    void tensor_to_mat(at::Tensor& input, cv::Mat& output);

    /**
     * @brief Convert a latent distribution to a pair of PyTorch tensors.
     *
     * This method converts the mean and standard deviation vectors of a latent
     * representation to PyTorch tensors. It assumes the input latent representation
     * is valid and has the correct dimensions.
     *
     * @param latent The input latent representation.
     * @param mu The output PyTorch tensor for the mean vector.
     * @param sigma The output PyTorch tensor for the standard deviation vector.
     */
    void latent_dist_to_tensors(latent_representation* latent, at::Tensor& mu, at::Tensor& sigma);

    /**
     * @brief Convert a pair of PyTorch tensors to a latent distribution
     * representation.
     *
     * This method converts the mean and standard deviation tensors to a latent
     * representation. It assumes the input tensors are valid and have the
     * correct dimensions.
     *
     * @param mu The input PyTorch tensor for the mean vector.
     * @param sigma The input PyTorch tensor for the standard deviation vector.
     * @param latent The output latent representation.
     */
    void tensors_to_latent_dist(at::Tensor& mu, at::Tensor& sigma, latent_representation* latent);

    /**
     * @brief Convert a latent representation to a single tensor via sampling.
     *
     * This method samples from the latent distribution defined by the mean and
     * standard deviation vectors of the input latent representation, and then
     * returns a single tensor containing the sampled values.
     *
     * @param latent The input latent representation.
     * @param output The output PyTorch tensor.
     */
    void latent_to_tensor(latent_representation* latent, at::Tensor& output);

    torch::jit::script::Module* module;

public:
    torch_module_wrapper();
    torch_module_wrapper(std::string traced_script_path);
    ~torch_module_wrapper();
    void load_traced_script(std::string traced_script_path);

    /**
     * @brief Forward pass through the model.
     */
    cv::Mat forward(cv::Mat& input);
};


class vae_base_model_wrapper : public torch_module_wrapper
{
public:
    vae_base_model_wrapper();
    vae_base_model_wrapper(std::string traced_script_path);
    ~vae_base_model_wrapper();

    void encode(cv::Mat& input, latent_representation* latent);
    void decode(latent_representation* latent, cv::Mat& output);
};

class vae_vcd_model_wrapper : public torch_module_wrapper
{
public:
    vae_vcd_model_wrapper();
    vae_vcd_model_wrapper(std::string traced_script_path);
    ~vae_vcd_model_wrapper();

    void encode(latent_representation* input, latent_representation* latent);
    void decode(latent_representation* latent, latent_representation* output);
};
