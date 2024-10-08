#include "torch_module_wrapper.h"


///////////////////////////
// TORCH_MODULE_WRAPPER //
/////////////////////////

torch_module_wrapper::torch_module_wrapper()
{
    module = NULL;
}

torch_module_wrapper::torch_module_wrapper(std::string traced_script_path)
{
    module = NULL;
    load_traced_script(traced_script_path);
}

void torch_module_wrapper::load_traced_script(std::string traced_script_path)
{
    if (module != NULL) {
        delete module;
    }

    module = new torch::jit::Module(torch::jit::load(traced_script_path));

    std::vector<torch::jit::Method> methods = module->get_methods();
    module->eval();
}

torch_module_wrapper::~torch_module_wrapper()
{
    delete module;
}

void torch_module_wrapper::mat_to_tensor(cv::Mat& input, at::Tensor& output)
{
    cv::Mat input_float;
    input.convertTo(input_float, CV_32F);
    cv::Mat input_normalized;
    cv::normalize(input_float, input_normalized, 0.0, 1.0, cv::NORM_MINMAX);
    at::Tensor tensor = torch::from_blob(input_normalized.data, {1, input_normalized.rows, input_normalized.cols, input_normalized.channels()});
    tensor = tensor.permute({0, 3, 1, 2});
    output = tensor.clone();
}

void torch_module_wrapper::tensor_to_mat(at::Tensor& input, cv::Mat& output)
{
    at::Tensor tensor = input.clone();
    tensor = tensor.squeeze();
    tensor = tensor.permute({1, 2, 0});
    tensor = tensor.contiguous();
    cv::Mat result(tensor.size(0), tensor.size(1), CV_32FC(tensor.size(2)));
    std::memcpy((void*)result.data, tensor.data_ptr(), sizeof(float) * tensor.numel());
    output = result;
}

void torch_module_wrapper::latent_dist_to_tensors(latent_representation* latent, at::Tensor& mu, at::Tensor& sigma)
{
    std::vector<double>* mu_vec = latent->get_mu();
    std::vector<double>* sigma_vec = latent->get_sigma();
    at::Tensor mu_tensor = torch::from_blob(mu_vec->data(), {1, mu_vec->size()});
    at::Tensor sigma_tensor = torch::from_blob(sigma_vec->data(), {1, sigma_vec->size()});
    mu = mu_tensor.clone();
    sigma = sigma_tensor.clone();
}

void torch_module_wrapper::tensors_to_latent_dist(at::Tensor& mu, at::Tensor& sigma, latent_representation* latent)
{
    std::vector<double>* mu_vec = new std::vector<double>(mu.data_ptr<float>(), mu.data_ptr<float>() + mu.numel());
    std::vector<double>* sigma_vec = new std::vector<double>(sigma.data_ptr<float>(), sigma.data_ptr<float>() + sigma.numel());
    latent->set_mu(mu_vec);
    latent->set_sigma(sigma_vec);
}

void torch_module_wrapper::latent_to_tensor(latent_representation* latent, at::Tensor& output)
{
    at::Tensor sample_tensor;
    std::vector<double>* sample = latent->sample(new std::vector<double>());
    sample_tensor = torch::from_blob(sample->data(), {1, sample->size()});
    output = sample_tensor.clone();
}


cv::Mat torch_module_wrapper::forward(cv::Mat& input)
{
    at::Tensor input_tensor;
    mat_to_tensor(input, input_tensor);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);
    at::Tensor output_tensor = module->forward(inputs).toTensor();
    cv::Mat output;
    tensor_to_mat(output_tensor, output);
    return output;
}


/////////////////////////////
// VAE_BASE_MODEL_WRAPPER //
///////////////////////////

vae_base_model_wrapper::vae_base_model_wrapper()
{
    module = NULL;
}

vae_base_model_wrapper::vae_base_model_wrapper(std::string traced_script_path)
{
    module = NULL;
    load_traced_script(traced_script_path);
}

vae_base_model_wrapper::~vae_base_model_wrapper()
{
    delete module;
}

void vae_base_model_wrapper::encode(cv::Mat& input, latent_representation* latent)
{
    at::Tensor input_tensor;
    mat_to_tensor(input, input_tensor);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);
    torch::jit::Method encode_method = module->get_method("encode");
    std::vector<torch::jit::IValue> output = encode_method(inputs).toTuple()->elements();
    at::Tensor mu_tensor = output[0].toTensor();
    at::Tensor sigma_tensor = output[1].toTensor();
    tensors_to_latent_dist(mu_tensor, sigma_tensor, latent);
}

void vae_base_model_wrapper::decode(latent_representation* latent, cv::Mat& output)
{
    at::Tensor latent_tensor;
    latent_to_tensor(latent, latent_tensor);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(latent_tensor);
    torch::jit::Method decode_method = module->get_method("decode");
    at::Tensor output_tensor = decode_method(inputs).toTensor();
    tensor_to_mat(output_tensor, output);
}


////////////////////////////
// VAE_VCD_MODEL_WRAPPER //
//////////////////////////

vae_vcd_model_wrapper::vae_vcd_model_wrapper()
{
    module = NULL;
}

vae_vcd_model_wrapper::vae_vcd_model_wrapper(std::string traced_script_path)
{
    module = NULL;
    load_traced_script(traced_script_path);
}

vae_vcd_model_wrapper::~vae_vcd_model_wrapper()
{
    delete module;
}

void vae_vcd_model_wrapper::encode(latent_representation* input, latent_representation* latent) {
    at::Tensor input_tensor;
    latent_to_tensor(input, input_tensor);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);
    std::vector<torch::jit::IValue> output = module->get_method("encode")(inputs).toTuple()->elements();
    at::Tensor mu_tensor = output[0].toTensor();
    at::Tensor sigma_tensor = output[1].toTensor();
    tensors_to_latent_dist(mu_tensor, sigma_tensor, latent);
}

void vae_vcd_model_wrapper::decode(latent_representation* latent, latent_representation* output) {
    at::Tensor latent_tensor;
    latent_to_tensor(latent, latent_tensor);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(latent_tensor);
    std::vector<torch::jit::IValue> latent_tensors = module->get_method("decode")(inputs).toTuple()->elements();
    at::Tensor mu_tensor = latent_tensors[0].toTensor();
    at::Tensor sigma_tensor = latent_tensors[1].toTensor();
    tensors_to_latent_dist(mu_tensor, sigma_tensor, output);
}
