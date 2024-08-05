#ifndef SVS_VISUAL_MEMORY
#define SVS_VISUAL_MEMORY

// C++ STD libraries
#include <string>
#include <vector>
#include <unordered_map>
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
class visual_long_term_memory
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

public:
    /**
     * @brief Convenient type reference for the archetypes held in this instance
     * of visual memory.
     *
     */
    typedef atype_T<img_T> archetype_T;

    /**
     * @brief Constructor for the visual_memory class.
     *
     * @param svs_parent The svs instance which manages this visual memory
     * instance.
     *
     */
    visual_long_term_memory(svs* svs_parent);

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
     * visual_concept_descriptor stored in _archetypes and calls its recognize method on
     * the given percept. The archetype with the highest resulting value is used
     * as the match.
     *
     * @param percept The percept to be identified.
     * @param output A `vmem_match` object to write the best match into.
     */
    void match(img_T* percept, vmem_match* output);

        /**
     * @brief Given a percept and comparison function, return a likely entity type/token.
     *
     * @details
     * Search through all of the entity - percept knowledge pairs in visual
     * memory and return the entity identifier most likely to correctly identify
     * what the percept is. This method just iterates through each
     * visual_concept_descriptor stored in _archetypes and calls the given recognize method on
     * the given percept. The archetype with the highest resulting value is used
     * as the match.
     *
     * @param percept The percept to be identified.
     * @param cmp_func A function which, given two percepts, compares them and
     *      returns their similarity as a double where higher is better
     * @param output A `vmem_match` object to write the best match into.
     */
    void match(img_T* percept, double (*cmp_func)(img_T*, img_T*), vmem_match* output);

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
};

#endif
