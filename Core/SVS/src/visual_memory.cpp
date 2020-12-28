// C++ STD libraries
#include <string>
#include <vector>
#include <unordered_map>
#include <exception>
// SVS Includes
#include "image.h"
#include "visual_memory.h"
#include "visual_archetype.h"
#include "exact_visual_archetype.h"

template <typename img_T, template<typename T> class atype_T>
visual_memory<img_T, atype_T>::visual_memory(svs* svs_parent) {
    _svs = svs_parent;
    _archetypes = std::vector<archetype_T*>();
    _id_index_map = std::unordered_map<std::string, int>();
}

template <typename img_T, template<typename T> class atype_T>
void visual_memory<img_T, atype_T>::store_percept(img_T* percept, std::string entity_id) {
    // Check if entity already had an entry
    if (_id_index_map.find(entity_id) != _id_index_map.end()) {
        // If so, update the existing entry with the new percept
        int target_idx = _id_index_map[entity_id];
        archetype_T* target_archetype = _archetypes.at(target_idx);
        target_archetype->store_percept(*percept);
    } else {
        // Otherwise, create a new entry for this percept
        int new_idx = (int)_archetypes.size();
        archetype_T* new_archetype = new archetype_T(entity_id);
        new_archetype->store_percept(*percept);
        _archetypes.push_back(new_archetype);
        _id_index_map[entity_id] = new_idx;
    }
}

template <typename img_T, template<typename T> class atype_T>
void visual_memory<img_T, atype_T>::recall(std::string entity_id, img_T* output) {
    int target_idx = _id_index_map[entity_id];
    archetype_T* target_archetype = _archetypes.at(target_idx);
    target_archetype->reconstruct(output);
}

template <typename img_T, template<typename T> class atype_T>
void visual_memory<img_T, atype_T>::match(img_T* percept, vmem_match* output) {
    throw "Not implemented yet";
}

template <typename img_T, template<typename T> class atype_T>
void visual_memory<img_T, atype_T>::search(img_T* percept, float threshold, std::vector<vmem_match*>* output) {
    throw "Not implemented yet";
}


// Explicit definition of visual_memory::match and ::search methods
template <>
void visual_memory<opencv_image, exact_visual_archetype>::match(opencv_image* percept, vmem_match* output) {
    std::string best_match_name;
    float best_similarity = 0.0;

    std::string current_id;
    float current_similarity;

    std::vector<exact_visual_archetype<opencv_image>*>::iterator atype_iterator;
    for (atype_iterator = _archetypes.begin(); atype_iterator != _archetypes.end(); atype_iterator++) {
        exact_visual_archetype<opencv_image>* archetype = *atype_iterator;
        opencv_image vmem_percept = archetype->get_raw_percept();
        
        archetype->get_id(current_id);
        current_similarity = percept->compare(&vmem_percept); 
        printf("Similarity of %s: %f\n", current_id.c_str(), current_similarity);

        if (current_similarity > best_similarity) { 
            best_similarity = current_similarity;
            best_match_name.assign(current_id);
        }
    }

    output->entity_id = best_match_name;
    output->confidence = best_similarity/3;  // div 3 because similarity metric is 0-3
}


// Explicit instantiation of visual memory classes
template class visual_memory<basic_image, exact_visual_archetype>;
#ifdef ENABLE_OPENCV
template class visual_memory<opencv_image, exact_visual_archetype>;
#endif
#ifdef ENABLE_ROS
template class visual_memory<pcl_image, exact_visual_archetype>;
#endif
