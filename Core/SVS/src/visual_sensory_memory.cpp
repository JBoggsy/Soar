#include <time.h>

#include "svs.h"
#include "visual_sensory_memory.h"
#include "image.h"
#include "symbol.h"
#include "soar_interface.h"

const std::string visual_sensory_memory::ROS_TOPIC_NAME = "vsm";

visual_sensory_memory::visual_sensory_memory(svs* _svs_ptr, soar_interface* _si)
{
    svs_ptr = _svs_ptr;
    svs_ptr->get_ros_interface()->create_rgb_publisher(ROS_TOPIC_NAME);
    si = _si;
    vsm_link = NULL;
    updated_link = NULL;
    update_counter = 0;
}

visual_sensory_memory::~visual_sensory_memory()
{
}

void visual_sensory_memory::add_wm_link(Symbol* _vsm_link) {
    vsm_link = _vsm_link;
}

void visual_sensory_memory::update_percept_buffer(const cv::Mat& new_image) {
    percept_buffer[0] = new opencv_image();
    percept_buffer[0]->update_image(new_image);
    update_counter++;

    if (vsm_link != NULL) {
        if (updated_link != NULL) {
            si->remove_wme(updated_link);
        }
        updated_link = si->make_wme(vsm_link, "updated", update_counter);
    }

    draw_percept_buffer();
}

void visual_sensory_memory::draw_percept_buffer() {
    percept_buffer[0]->draw_image("percept_buffer.png");
    svs_ptr->get_ros_interface()->publish_rgb_image(ROS_TOPIC_NAME, *percept_buffer[0]->get_image());
}
