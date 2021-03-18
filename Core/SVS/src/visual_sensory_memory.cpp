#include "svs.h"
#include "visual_sensory_memory.h"
#include "image.h"

const std::string visual_sensory_memory::ROS_TOPIC_NAME = "vsm";

visual_sensory_memory::visual_sensory_memory(svs* svs_ptr)
{
    _svs_ptr = svs_ptr;
    _svs_ptr->get_ros_interface()->create_rgb_publisher(ROS_TOPIC_NAME);
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
    _svs_ptr->get_ros_interface()->publish_rgb_image(ROS_TOPIC_NAME, *percept_buffer[0]->get_image());
}
