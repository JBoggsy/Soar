// C++ STD libraries
#include <string>
#include <vector>
#include <list>
#include <unordered_map>
#include <exception>
// SVS Includes
#include "image.h"
#include "visual_long_term_memory.h"
#include "visual_concept_descriptor.h"
#include "exact_visual_concept_descriptor.h"

template <typename img_T, template<typename T> class atype_T>
visual_long_term_memory<img_T, atype_T>::visual_long_term_memory(svs* svs_parent) {
    _svs = svs_parent;
    _archetypes = std::vector<archetype_T*>();
    _id_index_map = std::unordered_map<std::string, int>();
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::store_percept(img_T* percept, std::string entity_id) {
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
void visual_long_term_memory<img_T, atype_T>::recall(std::string entity_id, img_T* output) {
    int target_idx = _id_index_map[entity_id];
    archetype_T* target_archetype = _archetypes.at(target_idx);
    target_archetype->generate(output);
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::match(img_T* percept, vmem_match** output) {
    match(percept, output, 1);
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::match(img_T* percept, vmem_match** output, int n) {
    throw "Not implemented yet";
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::search(img_T* percept, float threshold, std::vector<vmem_match*>* output) {
    throw "Not implemented yet";
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::proxy_get_children(std::map<std::string, cliproxy*>& c) {
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VLTM INTERFACE ==========" << std::endl;
    os << "CLI USAGE:" << std::endl << std::endl;
    os << "====================================" << std::endl;
}

#ifdef ENABLE_OPENCV
////////////////////
// EXACT VCD VLTM //
////////////////////

template <>
void visual_long_term_memory<opencv_image, exact_visual_concept_descriptor>::match(opencv_image* percept, vmem_match** output, int n) {
    std::list<std::pair<float, std::string>> best_matches;

    std::string current_id;
    float current_similarity;

    std::vector<exact_visual_concept_descriptor<opencv_image>*>::iterator atype_iterator;
    for (atype_iterator = _archetypes.begin(); atype_iterator != _archetypes.end(); atype_iterator++) {
        exact_visual_concept_descriptor<opencv_image>* vcd = *atype_iterator;

        vcd->get_id(current_id);
        current_similarity = vcd->recognize(*percept);
        printf("Similarity of %s: %f\n", current_id.c_str(), current_similarity);

        std::list<std::pair<float, std::string>>::iterator best_match_itr;
        best_match_itr = best_matches.begin();
        int i = 0;
        for (; best_match_itr != best_matches.end(); best_match_itr++) {
            if (i >= n) { break; }
            if (current_similarity > best_match_itr->first) {
                best_matches.insert(best_match_itr, std::pair<float, std::string>(current_similarity, current_id));
                break;
            }
            i++;
        }
    }

    for (int i=0; i<n; i++) {
        std::pair<float, std::string> match = best_matches.front();
        float match_confidence = match.first;
        std::string match_id = match.second;
        output[i]->entity_id = match_id;
        output[i]->confidence = match_confidence;
        best_matches.pop_front();
    }
}

template class visual_long_term_memory<opencv_image, exact_visual_concept_descriptor>;
#endif

