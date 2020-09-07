// C++ STD libraries
#include <string>
#include <vector>
#include <unordered_map>
#include <exception>
// SVS Includes
#include "visual_memory.h"

visual_memory::visual_memory(svs* svs_parent) {
    _svs = svs_parent;
    _archetypes = std::vector<visual_archetype*>();
    _id_index_map = std::unordered_map<std::string, int>();
}

void visual_memory::store_percept(image_base* percept, std::string entity_id) {
    throw "Not implemented yet";
}

void visual_memory::recall(std::string entity_id, image_base* output) {
    throw "Not implemented yet";
}

std::string visual_memory::match(image_base* percept) {
    throw "Not implemented yet";
}

std::vector<std::string> visual_memory::search(image_base* percept, float threshold) {
    throw "Not implemented yet";
}