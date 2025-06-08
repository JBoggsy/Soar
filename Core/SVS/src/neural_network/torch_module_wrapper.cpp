#ifdef ENABLE_TORCH
#include "torch_module_wrapper.h"
#include <cstdio>

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
    cv::normalize(input_float, input_normalized, -1.0, 1.0, cv::NORM_MINMAX);
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
    cv::Mat result(tensor.size(0), tensor.size(1), CV_32FC(tensor.size(2)), tensor.data_ptr());
    cv::normalize(result, result, 0, 1, cv::NORM_MINMAX);
    result.copyTo(output);
    output.convertTo(output, CV_32F, 255);
}

void torch_module_wrapper::tensor_to_vector(at::Tensor& input, cv::Mat& output)
{
    at::Tensor tensor = input.squeeze();
    tensor = tensor.contiguous();
    int dims = tensor.size(0);
    output = cv::Mat(1, dims, CV_32F, tensor.data_ptr<float>()).clone();
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
    sample_tensor = torch::from_blob(sample->data(), {1, sample->size()}, {1, 1}, at::kDouble);
    sample_tensor = sample_tensor.to(at::kFloat);
    output = sample_tensor.clone();
}

void torch_module_wrapper::tensor_to_token_sequence(at::Tensor& input, token_sequence* output)
{
    int num_tokens = input.size(1);
    int num_features = input.size(2);
    cv::Mat tokens(num_tokens, num_features, CV_32F, input.data_ptr<float>());
    output->set_tokens(tokens);
}

void torch_module_wrapper::token_sequence_to_tensor(token_sequence* input, at::Tensor& output)
{
    cv::Mat tokens = input->get_tokens();
    at::Tensor tensor = torch::from_blob(tokens.data, {1, tokens.rows, tokens.cols}, at::kFloat);
    output = tensor.clone();
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

void torch_module_wrapper::print_tensor(at::Tensor& tensor)
{
    std::cout << "Tensor: " << tensor << std::endl;
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


/////////////////////////
// IMAGE FACTORY JEPA //
///////////////////////
img_factory_jepa_wrapper::img_factory_jepa_wrapper()
{
    module = NULL;
}
img_factory_jepa_wrapper::img_factory_jepa_wrapper(std::string traced_script_path)
{
    module = NULL;
    load_traced_script(traced_script_path);
}
img_factory_jepa_wrapper::~img_factory_jepa_wrapper()
{
    delete module;
}

void img_factory_jepa_wrapper::encode(cv::Mat& input, token_sequence* tokens)
{
    input = _pad_image(input);
    at::Tensor input_tensor;
    mat_to_tensor(input, input_tensor);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);
    torch::jit::Method encode_method = module->get_method("encode");
    at::Tensor output_tensor = encode_method(inputs).toTensor();
    tensor_to_token_sequence(output_tensor, tokens);
}

void img_factory_jepa_wrapper::summarize(token_sequence* tokens, cv::Mat& output)
{
    at::Tensor input_tensor;
    token_sequence_to_tensor(tokens, input_tensor);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);
    torch::jit::Method summarize_method = module->get_method("summarize");
    at::Tensor output_tensor = summarize_method(inputs).toTensor();
    tensor_to_vector(output_tensor, output);
}

void img_factory_jepa_wrapper::decode(token_sequence* tokens, cv::Mat& output)
{
    at::Tensor input_tensor;
    token_sequence_to_tensor(tokens, input_tensor);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);
    torch::jit::Method decode_method = module->get_method("decode");
    at::Tensor output_tensor = decode_method(inputs).toTensor();
    tensor_to_mat(output_tensor, output);
    output = _simplify_image(output);
}

void img_factory_jepa_wrapper::deobscure(token_sequence* source, token_sequence* conditioning, token_sequence* output)
{
    at::Tensor source_tensor;
    token_sequence_to_tensor(source, source_tensor);
    at::Tensor conditioning_tensor;
    token_sequence_to_tensor(conditioning, conditioning_tensor);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(source_tensor);
    inputs.push_back(conditioning_tensor);
    torch::jit::Method predict_method = module->get_method("predict");
    at::Tensor output_tensor = predict_method(inputs).toTensor();
    tensor_to_token_sequence(output_tensor, output);
}

void img_factory_jepa_wrapper::extract(token_sequence* source, token_sequence* base, token_sequence* output)
{
    at::Tensor source_tensor;
    token_sequence_to_tensor(source, source_tensor);
    at::Tensor base_tensor;
    token_sequence_to_tensor(base, base_tensor);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(source_tensor);
    inputs.push_back(base_tensor);
    torch::jit::Method extract_method = module->get_method("segment");
    at::Tensor output_tensor = extract_method(inputs).toTensor();
    tensor_to_token_sequence(output_tensor, output);
}

double img_factory_jepa_wrapper::get_shape_distance(token_sequence* a, token_sequence* b) {
    throw std::runtime_error("get_shape_distance is not implemented for img_factory_jepa_wrapper.");
}

cv::Mat img_factory_jepa_wrapper::_pad_image(const cv::Mat& image) {
    cv::Mat padded_image;
    if (image.empty()) {
        padded_image = cv::Mat(IMG_SIZE, IMG_SIZE, CV_8UC3, cv::Scalar(0, 0, 0));
    } else {
        padded_image = image.clone();
    }
    int top = 0, bottom = 0, left = 0, right = 0;
    int height = padded_image.rows;
    int width = padded_image.cols;
    if (height < IMG_SIZE) {
        int pad = IMG_SIZE - height;
        top = pad / 2;
        bottom = pad - top;
    }
    if (width < IMG_SIZE) {
        int pad = IMG_SIZE - width;
        left = pad / 2;
        right = pad - left;
    }
    cv::copyMakeBorder(padded_image, padded_image, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));

    int y = (padded_image.rows - IMG_SIZE) / 2;
    int x = (padded_image.cols - IMG_SIZE) / 2;
    padded_image = padded_image(cv::Rect(x, y, IMG_SIZE, IMG_SIZE));
    return padded_image;
}

cv::Mat img_factory_jepa_wrapper::_simplify_image(const cv::Mat& image) {
    cv::Mat simplified_image;
    // Apply median blur to reduce noise
    cv::medianBlur(image, simplified_image, 3);

    // Threshold image to remove low-intensity pixels
    cv::threshold(image, simplified_image, 0.25*255, 1.0*255, cv::THRESH_TOZERO);

    // Set transparent pixels to black
    cv::Mat mask = simplified_image > 0;
    simplified_image.setTo(cv::Scalar(0, 0, 0), ~mask);

    return simplified_image;
}

#endif
