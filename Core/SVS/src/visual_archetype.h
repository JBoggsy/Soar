#ifndef SVS_VISUAL_ARCHETYPE_H
#define SVS_VISUAL_ARCHETYPE_H

// C++ STD libraries
#include <string>


/**
 * @brief A visual_archetype instance holds visual knowledge of a particular
 * entity type or token.
 * 
 * @details A visual_archetype instance holds visual knowledge related to a 
 * particular kind of entity or a specific individual entity. The former we call
 * "entity types" and the latter "entity tokens". Examples of entity types 
 * include "person", "building", and "word". Examples of entity include "John 
 * Laird", "BBB Building", and "soar".
 * 
 * visual_archetype is an abstract class which makes no commitment to *how*
 * visual knowledge is represented. Instead, it provides an interface which
 * concrete sub-classes are guaranteed to implement. Different approaches to
 * visual knowledge representation can be used by creating concrete sub-classes
 * which each commit to one particular representation. 
 * 
 * visual_archetype and its concrete implementations are all templated, allowing
 * specialization for different types of visual input, including OpenCV and PCL
 * images.
 * 
 * visual_archetype instances only exist and are exclusively managed and 
 * accessed by the visual_memory instance connected to the agent.
 * 
 * @sa visual_memory
 * @sa image_base
 * @sa opencv_image
 * @sa pcl_image
 */
template<typename img_t>
class visual_archetype {
protected:
    /**
     * @brief An identifier for the entity type/token this instance represents. 
     * 
     */
    std::string _entity_id;
public:
    /**
     * @brief Store a new percept in the archetype.
     * 
     * @details Given a new visual percept, integrate it into the visual_archetype
     * instance. The exact manner of integration depends on the implementation
     * chosen by the concrete subclass.
     * 
     * @param percept The new visual percept to be integrated.
     */
    virtual void store_percept(img_t percept) = 0;

    /**
     * @brief Indicate how likely a percept is to contain this entity.
     * 
     * @details Given a visual percept, return a value between 0.0 and 1.0,
     * inclusive, indicating how likely it is the percept is an instance of the
     * entity or type of entity this archetype represents.
     * 
     * @param percept The visual percept which should be compared against.
     */
    virtual float compare(img_t percept) = 0;

    /**
     * @brief Generate a mental image of the entity this archetype represents.
     * 
     * @param output image_base which the reconstruction will be stored in.
     */
    virtual void reconstruct(img_t* output) = 0;
};

#endif