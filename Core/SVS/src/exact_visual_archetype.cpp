// C++ std libraries
#include <exception>
// SVS includes
#include "visual_archetype.h"
#include "exact_visual_archetype.h"
#include "image.h"


template<typename img_t>
exact_visual_archetype<img_t>::exact_visual_archetype(std::string entity_id) {
    _entity_id = entity_id;
    _percept = img_t();
}

template<typename img_t>
void exact_visual_archetype<img_t>::store_percept(img_t example) {
    _percept.copy_from(example);
}

template<typename img_t>
float exact_visual_archetype<img_t>::compare(img_t percept) {
    if (percept == _percept) { return 1.0f; }
    else { return 0.0f; }
}

template<typename img_t>
void exact_visual_archetype<img_t>::reconstruct(img_t* output) {
    output->copy_from(_percept);
}