#ifndef VISUAL_WORKING_MEMORY_H
#define VISUAL_WORKING_MEMORY_H

//////////////
// PREAMBLE //
//////////////
#include <map>

// SVS includes
///////////////
#include "visual_wme.h"

// forward declarations
///////////////////////
class svs;
class soar_interface;
class Symbol;
class opencv_image;
class visual_archetype;


/////////////////////////////////
// visual_working_memory CLASS //
/////////////////////////////////
class visual_working_memory
{
private:
    svs* svs_ptr;
    soar_interface* si;
    Symbol* vwm_link;

    std::map<int, visual_wme> vwmes;

    
    void _add_vwme(visual_wme new_vwme);


public:
    visual_working_memory(svs* svsp, soar_interface* _si);
    ~visual_working_memory();

    void add_vwme(visual_wme new_vwme);
    void add_image(opencv_image* new_image);
    void add_varch(visual_archetype* new_varch);

    void remove_vwme(visual_wme target);
    void remove_vwme(int target);

    void move_vwme(int target, float new_x, float new_y);
    void translate_vwme(int target, float d_x, float d_y);
    void rotate_vwme(int target, double rads);
    void flip_vwme_h(int target);
    void flip_vwme_v(int target);

    visual_wme* get_root_vwme();
};

#endif