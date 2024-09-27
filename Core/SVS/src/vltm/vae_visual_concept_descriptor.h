#pragma once
// SVS includes
#include "visual_concept_descriptor.h"
#include "latent_representation.h"
#include "neural_network.h"

template <typename img_t>
class vae_visual_concept_descriptor : public visual_concept_descriptor <img_t> {
protected:
    vae_vcd_model* archetype_model;
    std::string _entity_id;
public:
    vae_visual_concept_descriptor(std::string entity_id);
    ~vae_visual_concept_descriptor();
    void load_archetype_model(std::string traced_script_path);

    void store_percept(img_t example) override;
    double recognize(img_t percept) override;
    void generate(img_t* output) override;
};
