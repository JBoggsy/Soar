// C++ std libraries
#include <exception>
// SVS includes
#include "visual_archetype.h"
#include "exact_visual_archetype.h"
#include "image.h"


template<typename image_type>
exact_visual_archetype<image_type>::exact_visual_archetype(std::string entity_id) {
    _entity_id = entity_id;
    _percept = image_type();
}

template<typename image_type>
void exact_visual_archetype<image_type>::store_percept(image_type* example) {
    _percept.copy_from(example);
}

template<typename image_type>
float exact_visual_archetype<image_type>::compare(image_type* percept) {
    if (percept == _percept) { return 1.0f; }
    return _percept.compare(percept);
}

template<typename image_type>
void exact_visual_archetype<image_type>::reconstruct(image_type* output) {
    output->copy_from(_percept);
}