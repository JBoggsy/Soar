#include "vae_visual_concept_descriptor.h"


template<typename img_t>
vae_visual_concept_descriptor<img_t>::vae_visual_concept_descriptor(std::string entity_id) {
    _entity_id = entity_id;
    archetype_model = new vae_vcd_model();
    _example = new img_t();
}

template<typename img_t>
vae_visual_concept_descriptor<img_t>::~vae_visual_concept_descriptor() {
    delete archetype_model;
}

template<typename img_t>
void vae_visual_concept_descriptor<img_t>::load_archetype_model(std::string traced_script_path) {
    archetype_model->load_traced_script(traced_script_path);
}

template<typename img_t>
void vae_visual_concept_descriptor<img_t>::store_percept(img_t example) {
    throw std::runtime_error("vae_visual_concept_descriptor::store_percept not implemented");
}

template<>
double vae_visual_concept_descriptor<latent_representation>::recognize(latent_representation percept) {
    latent_representation encoded_latent;
    archetype_model->encode(&percept, &encoded_latent);
    std::vector<double> sample;
    encoded_latent.sample(&sample);
    double probability = sample.at(0);
    return probability;
}

template<>
void vae_visual_concept_descriptor<latent_representation>::generate(latent_representation* output) {
    std::vector<double> seed_mu = {1.0};
    std::vector<double> seed_sigma = {0.0};
    latent_representation seed = latent_representation(&seed_mu, &seed_sigma);
    archetype_model->decode(&seed, output);
}

template class vae_visual_concept_descriptor<latent_representation>;
