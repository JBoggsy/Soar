// C++ STD libraries
#include <string>
#include <vector>
#include <list>
#include <limits>
#include <unordered_map>
#include <exception>
// Third-party Includes
#include "base64.h"
// SVS Includes
#include "image.h"
#include "visual_long_term_memory.h"
#include "visual_concept_descriptor.h"
#include "exact_visual_concept_descriptor.h"
#ifdef ENABLE_TORCH
#include "latent_representation.h"
#include "token_sequence.h"
#include "vae_visual_concept_descriptor.h"
#include "jepa_visual_concept_descriptor.h"
#endif

/////////////////////
// TEMPLATE CLASS //
///////////////////

template <typename img_T, template<typename T> class atype_T>
visual_long_term_memory<img_T, atype_T>::visual_long_term_memory(svs* svs_parent) {
    _svs = svs_parent;
    _archetypes = std::vector<archetype_T*>();
    _id_index_map = std::unordered_map<std::string, int>();
    #ifdef ENABLE_TORCH
    _vcd_model = new img_factory_jepa();
    #endif
}


// VAE METHODS
//////////////

#ifdef ENABLE_TORCH
template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::load_vcd_model(std::string traced_script_path) {
    _vcd_model->load_traced_script(traced_script_path);
    if (!_vcd_model->get_module_loaded()) {
        throw std::runtime_error("Failed to load VAE model.");
    }
}

#ifdef ENABLE_OPENCV
template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::encode_image(opencv_image* input, img_T* representation) {
    _vcd_model->encode(*(input->get_image()), representation);
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::decode_representation(img_T* representation, opencv_image* output) {
    cv::Mat* output_image = new cv::Mat();
    _vcd_model->decode(representation, *output_image);
    output->set_image(output_image);
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::load_class_vcd_model(std::string vcd_id, std::string model_filepath) {
    throw std::runtime_error("Loading VCD models not possible with non-VAE VCDs.");
}
#endif
#endif

// MEMORY METHODS
/////////////////

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
    std::list<std::pair<double, std::string>> best_matches;
    best_matches.insert(best_matches.begin(), std::pair<double, std::string>(-1*std::numeric_limits<double>::infinity(), "NOMATCH"));

    std::string current_id;
    double current_similarity;

    typename std::vector<archetype_T*>::iterator vcd_itr = _archetypes.begin();
    for (; vcd_itr != _archetypes.end(); vcd_itr++) {
        archetype_T* vcd = *vcd_itr;

        vcd->get_id(current_id);
        current_similarity = vcd->recognize(*percept);

        std::list<std::pair<double, std::string>>::iterator best_match_itr;
        best_match_itr = best_matches.begin();
        int i = 0;
        for (; best_match_itr != best_matches.end(); best_match_itr++) {
            if (i >= n) { break; }
            if (current_similarity > best_match_itr->first) {
                best_matches.insert(best_match_itr, std::pair<double, std::string>(current_similarity, current_id));
                break;
            }
            i++;
        }
    }

    std::pair<double, std::string> match;
    double match_confidence;
    std::string match_id;
    vmem_match* vmatch;
    int i = 0;
    while (best_matches.size() > 0 && i < n) {
        match = best_matches.front();
        match_confidence = match.first;
        match_id = match.second;
        // printf("Match %d: %s (%f)\n", i, match_id.c_str(), match_confidence);
        vmatch = new vmem_match(match_id, match_confidence);
        output[i] = vmatch;
        best_matches.pop_front();
        i++;
    }
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::search(img_T* percept, float threshold, std::vector<vmem_match*>* output) {
    throw "Not implemented yet";
}


// CLI PROXY METHODS
////////////////////
template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    c["list"] = new memfunc_proxy<visual_long_term_memory<img_T, atype_T>>(this, &visual_long_term_memory<img_T, atype_T>::cli_list_vcd_ids);

    #ifdef ENABLE_TORCH
    c["load-vcd-model"] = new memfunc_proxy<visual_long_term_memory<img_T, atype_T>>(this, &visual_long_term_memory<img_T, atype_T>::cli_load_vcd_model);
    c["load-vcd-model"]->add_arg("TRACED_SCRIPT_PATH", "The path to the traced PyTorch script to load.");

    c["load-class-vcd"] = new memfunc_proxy<visual_long_term_memory<img_T, atype_T>>(this, &visual_long_term_memory<img_T, atype_T>::cli_load_class_vcd_model);
    c["load-class-vcd"]->add_arg("CLASS_NAME", "The name of the class (aka VCD ID) the model represents.");
    c["load-class-vcd"]->add_arg("MODEL_FILEPATH", "The path to the model file to load.");
    #endif

    c["learn"] = new memfunc_proxy<visual_long_term_memory<img_T, atype_T>>(this, &visual_long_term_memory<img_T, atype_T>::cli_learn);
    c["learn"]->add_arg("CLASS_NAME", "The name of the class (aka VCD ID) to create/update.");
    c["learn"]->add_arg("IMG_DATA", "The base64-encoded data of the image to update the VCD with.");

    c["generate"] = new memfunc_proxy<visual_long_term_memory<img_T, atype_T>>(this, &visual_long_term_memory<img_T, atype_T>::cli_generate);
    c["generate"]->add_arg("CLASS_NAME", "The name of the class (aka VCD ID) to generate an image of.");
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VLTM INTERFACE ==========" << std::endl;
    os << "VCDs: " << (int)_archetypes.size() << std::endl;
    os << "CLI USAGE:" << std::endl << std::endl;
    os << "svs vltm - Prints this message." << std::endl;
    os << "svs vltm.list - Prints a newline-separated list of the VCD IDs in VLTM." << std::endl;
    #ifdef ENABLE_TORCH
    os << "svs vltm.load-vcd-model <TRACED_SCRIPT_PATH> - Loads a traced PyTorch script into the VCD model." << std::endl;
    os << "svs vltm.load-class-vcd <CLASS_NAME> <MODEL_FILEPATH> - Loads a class-specific model for the given class name." << std::endl;
    #endif
    os << "svs vltm.learn <CLASS_NAME> <IMG_DATA> - Uses the image represented in <IMAGE_DATA> to update (or create) the VCD with the given class name. <IMAGE_DATA> should be a base64 encoding of the image." << std::endl;
    os << "svs vltm.generate <CLASS_NAME> - Generates an image of the VCD with the given class name, outputting it in base64. Empty string means no VCD with that class name exists." << std::endl;
    os << "====================================" << std::endl;
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::cli_list_vcd_ids(const std::vector<std::string>& args, std::ostream& os) {
    std::unordered_map<std::string, int>::iterator vcd_itr;
    std::string vcd_id;

    os << "Visual Concept Descriptors:" << std::endl;
    vcd_itr = _id_index_map.begin();
    for (; vcd_itr != _id_index_map.end(); vcd_itr++) {
        vcd_id = vcd_itr->first;
        os << vcd_id << std::endl;
    }

    return;
}

#ifdef ENABLE_TORCH
template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::cli_load_vcd_model(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "ERROR: No traced script path specified." << std::endl;
        return;
    }

    std::string traced_script_path(args[0]);
    load_vcd_model(traced_script_path);
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::cli_load_class_vcd_model(const std::vector<std::string>& args, std::ostream& os) {
    if (args.size() < 2) {
        os << "ERROR: Must specify class name and model filepath." << std::endl;
        return;
    }

    std::string vcd_id(args[0]);
    std::string model_filepath(args[1]);

    load_class_vcd_model(vcd_id, model_filepath);
}
#endif

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::cli_learn(const std::vector<std::string>& args, std::ostream& os) {
    if (args.size() < 2) {
        os << "ERROR: Must specify class name and image data." << std::endl;
        return;
    }

    std::string vcd_id(args[0]);
    std::string img_data(args[1]);
    std::string decoded_data = base64_decode(img_data);
    std::vector<uchar> data(decoded_data.begin(), decoded_data.end());
    cv::Mat raw_img = cv::imdecode(cv::Mat(data), -1);
    img_T* image = new img_T();
    image->update_image(raw_img);

    store_percept(image, vcd_id);
}

template <typename img_T, template<typename T> class atype_T>
void visual_long_term_memory<img_T, atype_T>::cli_generate(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "ERROR: No vcd class specified." << std::endl;
        return;
    }

    std::string vcd_id(args[0]);

    img_T* gened_image = new img_T();
    recall(vcd_id, gened_image);


    std::vector<uchar> raw_png_data;
    std::string b64_data;
    cv::imencode(std::string(".png"), *(gened_image->get_image()), raw_png_data);
    b64_data = base64_encode(raw_png_data.data(), raw_png_data.size());
    os << b64_data << std::endl;
}

#ifdef ENABLE_OPENCV
#ifndef ENABLE_TORCH
template class visual_long_term_memory<opencv_image, exact_visual_concept_descriptor>;
#endif

#ifdef ENABLE_TORCH

/////////////////////////////////
// EXACT LATENT SPECIALIZATION //
/////////////////////////////////
// TODO: Need to fix this later so that the choice of VCD model and its
// representation doesn't always break the specialization.
// template <>
// void visual_long_term_memory<latent_representation, exact_visual_concept_descriptor>::cli_learn(const std::vector<std::string>& args, std::ostream& os) {
//     if (args.size() < 2) {
//         os << "ERROR: Must specify class name and image data." << std::endl;
//         return;
//     }

//     std::string vcd_id(args[0]);
//     std::string img_data(args[1]);
//     std::string decoded_data = base64_decode(img_data);
//     std::vector<uchar> data(decoded_data.begin(), decoded_data.end());
//     cv::Mat raw_img = cv::imdecode(cv::Mat(data), -1);
//     opencv_image* image = new opencv_image();
//     image->update_image(raw_img);

//     // latent_representation* latent = new latent_representation();
//     _vcd_model->encode(*(image->get_image()), latent);

//     store_percept(latent, vcd_id);
// }

// template <>
// void visual_long_term_memory<latent_representation, exact_visual_concept_descriptor>::cli_generate(const std::vector<std::string>& args, std::ostream& os) {
//     if (args.empty()) {
//         os << "ERROR: No vcd class specified." << std::endl;
//         return;
//     }

//     std::string vcd_id(args[0]);

//     latent_representation* gened_latent = new latent_representation();
//     recall(vcd_id, gened_latent);

//     cv::Mat* gened_image_mat = new cv::Mat();
//     _vcd_model->decode(gened_latent, *(gened_image_mat));
//     opencv_image* gened_image = new opencv_image();
//     gened_image->set_image(gened_image_mat);


//     std::vector<uchar> raw_png_data;
//     std::string b64_data;
//     cv::imencode(std::string(".png"), *(gened_image->get_image()), raw_png_data);
//     b64_data = base64_encode(raw_png_data.data(), raw_png_data.size());
//     os << b64_data << std::endl;
// }

////////////////////////////
// VAE-VCD SPECIALIZATION //
////////////////////////////
// TODO: Need to fix this later so that the choice of VCD model and its
// representation doesn't always break the specialization.
// template <>
// void visual_long_term_memory<latent_representation, vae_visual_concept_descriptor>::load_class_vcd_model(std::string vcd_id, std::string model_filepath) {
//     archetype_T* target_archetype;
//     // Check if entity already had an entry
//     if (_id_index_map.find(vcd_id) != _id_index_map.end()) {
//         std::cout << "Updating existing VCD ID " << vcd_id << std::endl;
//         // If so, update the existing entry with the new percept
//         int target_idx = _id_index_map[vcd_id];
//         target_archetype = _archetypes.at(target_idx);
//     } else {
//         // Otherwise, create a new entry for this percept
//         int new_idx = (int)_archetypes.size();
//         target_archetype = new archetype_T(vcd_id);
//         _archetypes.push_back(target_archetype);
//         _id_index_map[vcd_id] = new_idx;
//     }
//     target_archetype->load_archetype_model(model_filepath);
// }

// template <>
// void visual_long_term_memory<latent_representation, vae_visual_concept_descriptor>::cli_learn(const std::vector<std::string>& args, std::ostream& os) {
//     if (args.size() < 2) {
//         os << "ERROR: Must specify class name and image data." << std::endl;
//         return;
//     }

//     std::string vcd_id(args[0]);
//     std::string img_data(args[1]);
//     std::string decoded_data = base64_decode(img_data);
//     std::vector<uchar> data(decoded_data.begin(), decoded_data.end());
//     cv::Mat raw_img = cv::imdecode(cv::Mat(data), -1);
//     opencv_image* image = new opencv_image();
//     image->update_image(raw_img);

//     latent_representation* latent = new latent_representation();
//     _vcd_model->encode(*(image->get_image()), latent);

//     store_percept(latent, vcd_id);
// }

// template <>
// void visual_long_term_memory<latent_representation, vae_visual_concept_descriptor>::cli_generate(const std::vector<std::string>& args, std::ostream& os) {
//     if (args.empty()) {
//         os << "ERROR: No vcd class specified." << std::endl;
//         return;
//     }

//     std::string vcd_id(args[0]);

//     latent_representation* gened_latent = new latent_representation();
//     recall(vcd_id, gened_latent);

//     cv::Mat* gened_image_mat = new cv::Mat();
//     _vcd_model->decode(gened_latent, *gened_image_mat);
//     opencv_image* gened_image = new opencv_image();
//     gened_image->set_image(gened_image_mat);

//     std::vector<uchar> raw_png_data;
//     std::string b64_data;
//     cv::imencode(std::string(".png"), *(gened_image->get_image()), raw_png_data);
//     b64_data = base64_encode(raw_png_data.data(), raw_png_data.size());
//     os << b64_data << std::endl;
// }

/////////////////////////////
// JEPA-VCD SPECIALIZATION //
/////////////////////////////
template <>
void visual_long_term_memory<token_sequence, jepa_visual_concept_descriptor>::load_class_vcd_model(std::string vcd_id, std::string model_filepath) {
}

template <>
void visual_long_term_memory<token_sequence, jepa_visual_concept_descriptor>::cli_learn(const std::vector<std::string>& args, std::ostream& os) {
    if (args.size() < 2) {
        os << "ERROR: Must specify class name and image data." << std::endl;
        return;
    }

    std::string vcd_id(args[0]);
    std::string img_data(args[1]);
    std::string decoded_data = base64_decode(img_data);
    std::vector<uchar> data(decoded_data.begin(), decoded_data.end());
    cv::Mat raw_img = cv::imdecode(cv::Mat(data), -1);
    opencv_image* image = new opencv_image();
    image->update_image(raw_img);

    token_sequence* latent = new token_sequence();
    _vcd_model->encode(*(image->get_image()), latent);

    store_percept(latent, vcd_id);
}

template <>
void visual_long_term_memory<token_sequence, jepa_visual_concept_descriptor>::cli_generate(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "ERROR: No vcd class specified." << std::endl;
        return;
    }

    std::string vcd_id(args[0]);

    token_sequence* gened_latent = new token_sequence();
    recall(vcd_id, gened_latent);

    cv::Mat* gened_image_mat = new cv::Mat();
    _vcd_model->decode(gened_latent, *gened_image_mat);
    opencv_image* gened_image = new opencv_image();
    gened_image->set_image(gened_image_mat);

    std::vector<uchar> raw_png_data;
    std::string b64_data;
    cv::imencode(std::string(".png"), *(gened_image->get_image()), raw_png_data);
    b64_data = base64_encode(raw_png_data.data(), raw_png_data.size());
    os << b64_data << std::endl;
}


// /////////////////////////////
// // EXPLICIT INSTANTIATIONS //
// /////////////////////////////
// template class visual_long_term_memory<latent_representation, exact_visual_concept_descriptor>;
// template class visual_long_term_memory<latent_representation, vae_visual_concept_descriptor>;
template class visual_long_term_memory<token_sequence, jepa_visual_concept_descriptor>;

#endif
#endif
