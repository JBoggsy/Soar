#pragma once
// standard includes
#include <string>
// third-party includes
#include <opencv2/opencv.hpp>
// SVS includes
#include "latent_representation.h"

// forward declarations
class torch_module_wrapper;
class vae_base_model_wrapper;
class vae_vcd_model_wrapper;


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
protected:
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
    neural_network() {}
    neural_network(std::string traced_script_path);
    ~neural_network();
    void load_traced_script(std::string traced_script_path);
    cv::Mat forward(cv::Mat& input);
};


class vae_base_model : public neural_network
{
protected:
    vae_base_model_wrapper* module;

public:
    vae_base_model() {}
    vae_base_model(std::string traced_script_path) : neural_network(traced_script_path) {}
    ~vae_base_model();

    void encode(cv::Mat& input, latent_representation* latent);
    void decode(latent_representation* latent, cv::Mat& output);
};


class vae_vcd_model : public neural_network
{
protected:
    vae_vcd_model_wrapper* module;
public:
    vae_vcd_model() {}
    vae_vcd_model(std::string traced_script_path) : neural_network(traced_script_path) {}
    ~vae_vcd_model();

    void encode(latent_representation* input, latent_representation* latent);
    void decode(latent_representation* latent, latent_representation* output);
};
