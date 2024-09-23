// local includes
#include "neural_network.h"
#include "torch_module_wrapper.h"

neural_network::neural_network()
{
}

neural_network::~neural_network()
{
}

void neural_network::load_traced_script(std::string traced_script_path)
{
    module = new torch_module_wrapper();
    module->load_traced_script(traced_script_path);
}

cv::Mat neural_network::forward(cv::Mat input)
{
    // convert cv::Mat to torch::Tensor
    torch::Tensor tensor = torch::from_blob(input.data, {input.rows, input.cols, 3}, torch::kByte);
    tensor = tensor.permute({2, 0, 1});
    tensor = tensor.toType(torch::kFloat);
    tensor = tensor.div(255);
    tensor = tensor.unsqueeze(0);

    // forward pass
    torch::Tensor output = module->forward(tensor);

    // convert torch::Tensor to cv::Mat
    output = output.squeeze(0);
    output = output.mul(255);
    output = output.toType(torch::kByte);
    output = output.permute({1, 2, 0});
    cv::Mat result(output.size(0), output.size(1), CV_8UC3, output.data_ptr());

    return result;
}