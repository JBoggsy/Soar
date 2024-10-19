#ifdef ENABLE_TORCH
// third-party includes
#include <opencv2/opencv.hpp>
// local includes
#include "neural_network.h"
#include "torch_module_wrapper.h"


/////////////////
// BASE CLASS //
///////////////

neural_network::neural_network()
{
    module = new torch_module_wrapper();
    module_loaded = false;
}

neural_network::neural_network(std::string traced_script_path)
{
    module = new torch_module_wrapper(traced_script_path);
    module_loaded = true;
}

neural_network::~neural_network()
{
    delete module;
}


void neural_network::load_traced_script(std::string traced_script_path)
{
    module->load_traced_script(traced_script_path);
    module_loaded = true;
    printf("neural_network: Loaded traced script from %s\n", traced_script_path.c_str());
}

cv::Mat neural_network::forward(cv::Mat& input)
{
    if (!module_loaded) {
        throw std::runtime_error("Tried to forward through a neural network without loading a traced script first.");
    }
    return module->forward(input);
}


/////////////////////
// VAE BASE MODEL //
///////////////////

vae_base_model::vae_base_model()
{
    module = new vae_base_model_wrapper();
    module_loaded = false;
}

vae_base_model::vae_base_model(std::string traced_script_path)
{
    module = new vae_base_model_wrapper(traced_script_path);
    module_loaded = true;
}

vae_base_model::~vae_base_model()
{
    delete module;
}

void vae_base_model::load_traced_script(std::string traced_script_path)
{
    module->load_traced_script(traced_script_path);
    module_loaded = true;
    printf("vae_base_model: Loaded traced script from %s\n", traced_script_path.c_str());
}

void vae_base_model::encode(cv::Mat& input, latent_representation* latent)
{
    module->encode(input, latent);
}

void vae_base_model::decode(latent_representation* latent, cv::Mat& output)
{
    module->decode(latent, output);
}


////////////////////
// VAE VCD MODEL //
//////////////////

vae_vcd_model::vae_vcd_model()
{
    module = new vae_vcd_model_wrapper();
    module_loaded = false;
}

vae_vcd_model::vae_vcd_model(std::string traced_script_path)
{
    module = new vae_vcd_model_wrapper(traced_script_path);
    module_loaded = true;
}

vae_vcd_model::~vae_vcd_model()
{
    delete module;
}

void vae_vcd_model::load_traced_script(std::string traced_script_path)
{
    module->load_traced_script(traced_script_path);
    module_loaded = true;
    printf("vae_vcd_model: Loaded traced script from %s\n", traced_script_path.c_str());
}

void vae_vcd_model::encode(latent_representation* input, latent_representation* latent)
{
    module->encode(input, latent);
}

void vae_vcd_model::decode(latent_representation* latent, latent_representation* output)
{
    module->decode(latent, output);
}

#endif