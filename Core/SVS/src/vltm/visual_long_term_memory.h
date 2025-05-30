#ifndef SVS_VISUAL_MEMORY
#define SVS_VISUAL_MEMORY

// C++ STD libraries
#include <string>
#include <vector>
#include <unordered_map>
// SVS Includes
#include "cliproxy.h"
#ifdef ENABLE_TORCH
#include "neural_network.h"
#include "jepa_visual_concept_descriptor.h"
#endif
// Forward declarations
class svs;  // "svs.h"
class image_base;  // "image.h"
class basic_image;
class opencv_image;
class pcl_image;
class archetype_base;  // "visual_concept_descriptor.h"


/**
 * @brief Encapsulates a result of an attempt to identify a percept via the
 * visual_memory::match or visual_memory::search methods.
 *
 */
typedef struct vmem_match {
    std::string entity_id;
    float confidence;

    vmem_match() {}
    vmem_match(std::string id, float conf) {
        entity_id = id;
        confidence = conf;
    }
} vmem_match;


/**
 * @brief
 * Encapsulates an agent's knowledge of visual percepts.
 *
 * @details
 * A visual_memory instance holds the total visual knowledge of a particular
 * agent,in the form of numerous visual_concept_descriptor instances. It provides look-up
 * access to the agent via a "command" link in working memory, and can learn
 * from visual input, working memory, and semantic memory. The class stores the
 * various visual_concept_descriptor instances it has learned or been given in the
 * archetypes attribute, and creates an archetype-identifier mapping using the
 * entityIDIndex attribute. The class also provides several methods for storing,
 * retrieving, or altering visual memories.
 *
 * @sa visual_concept_descriptor
 */
template <typename img_T, template<typename T> class atype_T>
class visual_long_term_memory : public cliproxy
{
private:
    /**
     * @brief Provides a mapping between string entity IDs and the index of the
     * relevant visual archetype in the _archetypes vector.
     *
     */
    std::unordered_map<std::string, int> _id_index_map;
    /**
     * @brief List of all of the agent's known visual archetypes.
     *
     */
    std::vector<atype_T<img_T>*> _archetypes;
    /**
     * @brief Maintains a connection to the svs instance.
     *
     * @sa svs
     */
    svs* _svs;

    #ifdef ENABLE_TORCH
    // TODO: For now I'm just commenting this out, but I should make this much
    // more flexible. Really the whole NN system here should be much more
    // flexible, so that it can be altered at run-time.
    // vae_base_model* _vcd_model;
    img_factory_jepa* _vcd_model;
    #endif

public:
    /**
     * @brief Convenient type reference for the archetypes held in this instance
     * of visual memory.
     */
    typedef atype_T<img_T> archetype_T;

    /**
     * @brief Constructor for the visual_memory class.
     *
     * @param svs_parent The svs instance which manages this visual memory
     * instance.
     */
    visual_long_term_memory(svs* svs_parent);

    /////////////////
    // VAE METHODS //
    /////////////////

    #ifdef ENABLE_TORCH
    /**
     * @brief Load a traced PyTorch script into the VCD model.
     *
     * @param traced_script_path The path to the traced PyTorch script to load.
     */
    void load_vcd_model(std::string traced_script_path);

    #ifdef ENABLE_OPENCV
    /**
     * @brief Encode an opencv_image into a VCD representation.
     *
     * @param input The image to encode.
     * @param representation The representation to write the result into.
     */
    void encode_image(opencv_image* input, img_T* representation);

    /**
     * @brief Decode a VCD representation into an opencv_image.
     *
     * @param latent The representation to decode.
     * @param output The opencv_image to write the result into.
     */
    void decode_representation(img_T* latent, opencv_image* output);

    /**
     * @brief Load a class-specific VCD model from a given filepath.
     *
     * @param vcd_id The string identifier of the VCD to load the model into.
     * @param model_filepath The path to the model file to load.
     */
    void load_class_vcd_model(std::string vcd_id, std::string model_filepath);

    img_factory_jepa* get_vcd_model() {
        return _vcd_model;
    }

    #endif
    #endif

    ////////////////////
    // MEMORY METHODS //
    ////////////////////

    /**
     * @brief
     * Store a given percept in visual memory.
     *
     * @details
     * Given a pointer to a basic_image instance and a ground truth entity
     * identifier, store the given percept in visual memory as an instance of
     * that kind of entity. If no such entity has been seen before, a new
     * visual_concept_descriptor instance is created. If the entity is already known, the
     * corresponding visual_concept_descriptor instance is updated with the new information.
     *
     * @param percept The percept to be stored.
     * @param entity_id The string identifier of the entity type or token the
     * percept is associated with.
     *
     * @sa basic_image
     * @sa visual_concept_descriptor
     */
    void store_percept(img_T* percept, std::string entity_id);

    /**
     * @brief Given a percept, return a likely entity type/token.
     *
     * @details
     * Search through all of the entity - percept knowledge pairs in visual
     * memory and return the entity identifier most likely to correctly identify
     * what the percept is. This method just iterates through each
     * visual_concept_descriptor stored in _archetypes and calls its recognize
     * method on the given percept. The archetype with the highest resulting
     * value is used as the match.
     *
     * @param percept The percept to be identified.
     * @param output An array of `vmem_match` objects to write the best matches
     * into. Expected to be of length `n`.
     * @param n (Optional, default 1) How many matches to return. Returns the
     * top `n` matches.
     */
    void match(img_T* percept, vmem_match** output);
    void match(img_T* percept, vmem_match** output, int n);


    /**
     * @brief Given the string identifier of a VisualArchetype instance presumed
     * to be in visual memory, create a new visual percept of that entity in the
     * specified pointer.
     *
     * @param entity_id The string identifier of the entity type/token to be
     * recalled.
     * @param output An basic_image pointer which will receive the result
     * of recalling the specified entity.
     */
    void recall(std::string entity_id, img_T* output);

    /**
     * @brief Given a percept and a confidence threshold, return a list of the
     * entity IDs which are believed to be contained in the percept with
     * confidence greater than the threshold.
     *
     * @param percept The percept to search in.
     * @param threshold The minimum confidence value an archetype must return
     * from its comapre method to be included in the results of this method.
     * @param output A vector to write the matches to.
     */
    void search(img_T* percept, float threshold, std::vector<vmem_match*>* output);

    //////////////////////
    // CLIPROXY METHODS //
    //////////////////////

    /**
     * @brief Provide a map of the sub-commands for this command to the CLI
     * parser.
     *
     * @details Todo
     *
     * @todo Write details
     *
     * @param c A mapping of string identifiers to `cliproxy` instance.
     */
    void proxy_get_children(std::map<std::string, cliproxy*>& c);

    /**
     * @brief Displays how many VCDs are in VLTM, then prints out the help menu.
     */
    void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);

    /**
     * @brief Prints a newline-separated list of the VCD IDs (aka class names)
     * in VLTM.
     */
    void cli_list_vcd_ids(const std::vector<std::string>& args, std::ostream& os);


    #ifdef ENABLE_TORCH
    /**
     * @brief Load a traced PyTorch script into the VAE model.
     */
    void cli_load_vcd_model(const std::vector<std::string>& args, std::ostream& os);

    /**
     * @brief Given a class name and a filepath to a torch model, create a
     * vcd with the given class name and load the model from the given filepath.
     */
    void cli_load_class_vcd_model(const std::vector<std::string>& args, std::ostream& os);
    #endif

    /**
     * @brief Given a class name and base64-encoded image data, update or create
     * the VCD in VLTM with the given class name using the image reconstructed
     * from the image data.
     */
    void cli_learn(const std::vector<std::string>& args, std::ostream& os);

    /**
     * @brief Given a class name, return base64-encoded image data corresponding
     * to a generated image of that class.
     */
    void cli_generate(const std::vector<std::string>& args, std::ostream& os);
};

#endif

#ifdef ENABLE_OPENCV
#ifdef ENABLE_TORCH
    // #define VLTM_TYPE visual_long_term_memory<latent_representation, exact_visual_concept_descriptor>
    // #define VLTM_TYPE visual_long_term_memory<latent_representation, vae_visual_concept_descriptor>
    #define VLTM_TYPE visual_long_term_memory<token_sequence, jepa_visual_concept_descriptor>
    // #define VLTM_TYPE visual_long_term_memory<opencv_image, exact_visual_concept_descriptor>
#else
    #define VLTM_TYPE visual_long_term_memory<opencv_image, exact_visual_concept_descriptor>
#endif
#endif
