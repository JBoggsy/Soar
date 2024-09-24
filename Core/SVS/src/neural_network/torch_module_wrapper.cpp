#include "torch_module_wrapper.h"

torch_module_wrapper::torch_module_wrapper()
{
}

torch_module_wrapper::torch_module_wrapper(std::string traced_script_path)
{
    load_traced_script(traced_script_path);
}

torch_module_wrapper::~torch_module_wrapper()
{
}

at::Tensor torch_module_wrapper::forward(at::Tensor input)
{
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input);
    return module.forward(inputs).toTensor();
}

void torch_module_wrapper::load_traced_script(std::string traced_script_path)
{
    module = torch::jit::load(traced_script_path);
}
