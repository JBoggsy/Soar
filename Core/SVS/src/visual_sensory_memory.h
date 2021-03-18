#ifndef VISUAL_SENSORY_MEMORY_H
#define VISUAL_SENSORY_MEMORY_H

//////////////
// PREAMBLE //
//////////////
// SVS includes
#include "image.h"
// forward definitions
class svs;

class visual_sensory_memory
{
private:
    svs* _svs_ptr;

    const static int PERCEPT_BUFFER_SIZE = 1;
    opencv_image* percept_buffer [PERCEPT_BUFFER_SIZE];

public:
    visual_sensory_memory(svs* svs_ptr);
    ~visual_sensory_memory();

    void update_percept_buffer(const cv::Mat& new_percept);
    void draw_percept_buffer();
};


#endif