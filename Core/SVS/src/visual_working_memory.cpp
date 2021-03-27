#include "soar_interface.h"
#include "svs.h"
#include "image.h"
#include "visual_working_memory.h"
#include "visual_wme.h"

visual_working_memory::visual_working_memory(svs* svsp, soar_interface* soar_int, Symbol* link) {
    svs_ptr = svsp;
    si = soar_int;
    vwm_link = link;
}

void visual_working_memory::_add_vwme(visual_wme new_vwme) {
    vwmes[new_vwme.get_id()] = new_vwme;
}

void visual_working_memory::add_vwme(visual_wme new_vwme) {
    _add_vwme(new_vwme);
}

void visual_working_memory::add_image(opencv_image* new_image) {
    // TODO: Implement
}

void visual_working_memory::add_varch(visual_archetype* new_varch) {
    // TODO: Implement
}

void visual_working_memory::remove_vwme(visual_wme target) {
    vwmes.erase(target.get_id());
}

void visual_working_memory::remove_vwme(std::string target) {
    vwmes.erase(target);
}
