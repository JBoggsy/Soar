#pragma once
// torchscript includes
#include <torch/script.h>


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
private:
    torch::jit::script::Module module;
public:
    torch_module_wrapper();
    torch_module_wrapper(std::string traced_script_path);
    ~torch_module_wrapper();
    at::Tensor forward(at::Tensor input);
    void load_traced_script(std::string traced_script_path);
};


class vae_model_wrapper : torch_module_wrapper
{
public:
    vae_model_wrapper(std::string traced_script_path) : torch_module_wrapper(traced_script_path) {}
    ~vae_model_wrapper() {}

    at::Tensor encode(at::Tensor input) {
        return module.encode(input);
    }

    at::Tensor decode(at::Tensor input)
    {
        return module.decode(input);
    }
};