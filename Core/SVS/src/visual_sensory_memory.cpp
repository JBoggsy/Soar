#include "visual_sensory_memory.h"
#include "image.h"

visual_sensory_memory::visual_sensory_memory(svs* svs_ptr)
{
    _svs_ptr = svs_ptr;
}

visual_sensory_memory::~visual_sensory_memory()
{
}

void visual_sensory_memory::update_percept_buffer(const cv::Mat& new_image) {
    percept_buffer[0] = new opencv_image();
    percept_buffer[0]->update_image(new_image);
    draw_percept_buffer();
}

void visual_sensory_memory::draw_percept_buffer() {
    percept_buffer[0]->draw_image("percept_buffer.png");
}