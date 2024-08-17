#ifndef SVS_EXACT_VISUAL_CONCEPT_DESCRIPTOR_H
#define  SVS_EXACT_VISUAL_CONCEPT_DESCRIPTOR_H
// C++ STD libraries
#include <string>
// SVS includes
#include "visual_concept_descriptor.h"

/**
 * @brief Represents visual knowledge exactly as it receives it.
 *
 * @details A visual_concept_descriptor instance holds visual knowledge related
 * to a particular kind of entity or a specific individual entity. The former we
 * call "entity types" and the latter "entity tokens". Examples of entity types
 * include "person", "building", and "word". Examples of entity include "John
 * Laird", "BBB Building", and "soar".
 *
 * The exact_visual_concept_descriptor class represents a visual concept using a
 * single archetypal image. Recognition is done by sliding the archetypal image
 * over the (padded if necessary) target image and computing L2 difference for
 * each window, then taking the lowest of these. Generation simply returns the
 * archetypal image.
 *
 * exact_visual_concept_descriptor is templated to allow specializing for
 * different types of visual input, including the openv_image and pcl_image
 * classes.
 *
 * @sa visual_memory
 * @sa image_base
 * @sa opencv_image
 * @sa pcl_image
 */
template<typename img_t>
class exact_visual_concept_descriptor : public visual_concept_descriptor<img_t> {
    protected:
        img_t archetype;
        /**
         * @brief An identifier for the entity type/token this instance represents.
         */
        std::string _entity_id;

    public:
        exact_visual_concept_descriptor(std::string entity_id);

        void get_id(std::string& result) { result.assign(_entity_id); }
        img_t get_raw_percept() { return archetype; }

        /**
         * @brief Store a new percept in the VCD.
         *
         * @details Given a new visual percept, integrate it into the
         * visual_concept_descriptor instance by cloning the percept data into the
         * archetype field of the VCD instance.
         *
         * @param percept The new visual percept to be integrated.
         */
        void store_percept(img_t example) override;

        /**
         * @brief Indicate whether the given percept is the same as this one.
         *
         * @details Given a visual percept, return a confidence score that the
         * percept contains an instance of the VCD.
         *
         * @param percept The visual percept which should be compared against.
         */
        double recognize(img_t percept) override;

        /**
         * @brief Generate a mental image of the entity this VCD represents.
         *
         * @details Clone the percept which is stored in the archetype field of
         * this VCD into the output.
         *
         * @param output Pointer to an instance of the correct image type img_t
         * which the reconstruction will be stored in.
         */
        void generate(img_t* output) override;
};

#endif
