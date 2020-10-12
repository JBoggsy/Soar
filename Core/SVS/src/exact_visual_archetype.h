#ifndef SVS_EXACT_VISUAL_ARCHETYPE
#define  SVS_EXACT_VISUAL_ARCHETYPE
// C++ STD libraries
#include <string>
// SVS includes
#include "visual_archetype.h"

/**
 * @brief Represents visual knowledge exactly as it receives it.
 * 
 * @details A visual_archetype instance holds visual knowledge related to a 
 * particular kind of entity or a specific individual entity. The former we call
 * "entity types" and the latter "entity tokens". Examples of entity types 
 * include "person", "building", and "word". Examples of entity include "John 
 * Laird", "BBB Building", and "soar".
 * 
 * The exact_visual_archetype class represents visual percepts exactly as they
 * are received, and does matching as a true/false comparison of equality. Thus
 * it lacks any amount of flexibility or subtlety and should not be used for
 * any real-world application.
 * 
 * exact_visual_archetype is templated to allow specializing for different types
 * of visual input, including the openv_image and pcl_image classes.
 * 
 * @sa visual_memory
 * @sa image_base
 * @sa opencv_image
 * @sa pcl_image
 */
template<typename img_t>
class exact_visual_archetype : public visual_archetype {
    protected:
        img_t _percept;
    
    public:
        exact_visual_archetype(std::string entity_id);

        /**
         * @brief Store a new percept in the archetype.
         * 
         * @details Given a new visual percept, integrate it into the 
         * visual_archetype instance by cloning the percept data into the
         * _percept field of the archetype instance.
         * 
         * @param percept The new visual percept to be integrated.
         */
        void store_percept(img_t example) override;

        /**
         * @brief Indicate whether the given percept is the same as this one.
         * 
         * @details Given a visual percept, return either 0.0 to indicate 
         * inequality or 1.0 to indicate equaity. 
         * 
         * @param percept The visual percept which should be compared against.
         */
        float compare(img_t percept) override;

        /**
         * @brief Generate a mental image of the entity this archetype represents.
         * 
         * @details Clone the percept which is stored in the _percept field of
         * this archetype into the output.
         * 
         * @param output Pointer to an instance of the correct image type img_t
         * which the reconstruction will be stored in.
         */
        void reconstruct(img_t* output) override;
};

#endif