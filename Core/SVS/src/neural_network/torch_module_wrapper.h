#pragma once
// torchscript includes
#include <torch/script.h>

class torch_module_wrapper
{
private:
    torch::jit::script::Module module;
public:
    torch_module_wrapper();
    ~torch_module_wrapper();
    at::Tensor forward(at::Tensor input);
    void load_traced_script(std::string traced_script_path);
};