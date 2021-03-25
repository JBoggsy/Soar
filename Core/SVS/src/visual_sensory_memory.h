#ifndef VISUAL_SENSORY_MEMORY_H
#define VISUAL_SENSORY_MEMORY_H

//////////////
// PREAMBLE //
//////////////

// standard lib includes
////////////////////////
#include <string>

// SVS includes
///////////////
#include "image.h"

// forward definitions
//////////////////////
class svs;


/////////////////////////////////
// visual_sensory_memory CLASS //
/////////////////////////////////
class visual_sensory_memory
{
private:
    const static std::string ROS_TOPIC_NAME;
    const static int PERCEPT_BUFFER_SIZE = 1;

    svs* _svs_ptr;
    opencv_image* percept_buffer [PERCEPT_BUFFER_SIZE];

public:
    visual_sensory_memory(svs* svs_ptr);
    ~visual_sensory_memory();

    void update_percept_buffer(const cv::Mat& new_percept);
    void draw_percept_buffer();
};

#endif