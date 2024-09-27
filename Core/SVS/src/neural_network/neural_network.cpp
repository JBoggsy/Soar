// third-party includes
#include <opencv2/opencv.hpp>
// local includes
#include "neural_network.h"
#include "torch_module_wrapper.h"


/////////////////
// BASE CLASS //
///////////////

neural_network::neural_network(std::string traced_script_path) { load_traced_script(traced_script_path); }
neural_network::~neural_network() { delete module; }


void neural_network::load_traced_script(std::string traced_script_path)
{
    module = new torch_module_wrapper(traced_script_path);
    module_loaded = true;
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

vae_base_model::~vae_base_model() { delete module; }

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

vae_vcd_model::~vae_vcd_model() { delete module; }

void vae_vcd_model::encode(latent_representation* input, latent_representation* latent)
{
    module->encode(input, latent);
}

void vae_vcd_model::decode(latent_representation* latent, latent_representation* output)
{
    module->decode(latent, output);
}
