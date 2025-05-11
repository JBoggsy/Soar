#ifdef ENABLE_TORCH
#pragma once
// SVS includes
#include "visual_concept_descriptor.h"
#include "token_sequence.h"
#include "neural_network.h"

template <typename img_t>
class jepa_visual_concept_descriptor : public visual_concept_descriptor <img_t> {
protected:
    std::string _entity_id;

    img_t* _example;
public:
    jepa_visual_concept_descriptor(std::string entity_id);
    ~jepa_visual_concept_descriptor();
    void get_id(std::string& result) { result.assign(_entity_id); }

    void store_percept(img_t example) override;
    double recognize(img_t percept) override;
    void generate(img_t* output) override;
};
#endif
