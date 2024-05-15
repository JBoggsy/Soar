#include <time.h>

#include "svs.h"
#include "visual_working_memory.h"
#include "image.h"
#include "symbol.h"
#include "soar_interface.h"
// Base64 library for image transfer
#include "base64.h"


#ifdef ENABLE_ROS
const std::string visual_working_memory::ROS_TOPIC_NAME = "vwm";
#endif

visual_working_memory::visual_working_memory(svs* svs_ptr, soar_interface* si, Symbol* vwm_link)
    : svs_ptr(svs_ptr), si(si), vwm_link(vwm_link)
{
    #ifdef ENABLE_ROS
    svs_ptr->get_ros_interface()->create_rgb_publisher(ROS_TOPIC_NAME);
    #endif
}

visual_working_memory::~visual_working_memory() {
}

visual_working_memory* visual_working_memory::clone(Symbol* vwm_link) {
    visual_working_memory* new_vwm = new visual_working_memory(svs_ptr, si, link);
}
