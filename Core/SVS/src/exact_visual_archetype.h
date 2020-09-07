#ifndef SVS_EXACT_VISUAL_ARCHETYPE
#define  SVS_EXACT_VISUAL_ARCHETYPE
// C++ STD libraries
#include <string>
// SVS includes
#include "visual_archetype.h"

/**
 * @brief Represents visual knowledge exactly as it receives it.
 * 
 * @details The template parameter indicates which subclass of image_base the
 * percept is stored as.
 */
template<typename image_type>
class exact_visual_archetype : public visual_archetype {
    protected:
        image_type* _percept;
    
    public:
        exact_visual_archetype(std::string entity_id);

        void store_percept(image_type* example);
        float compare(image_type* percept);
        void reconstruct(image_type* output);
};

#endif