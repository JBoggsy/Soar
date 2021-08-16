// C++ standard libraries
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
// OpenCV library
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
// SVS includes
#include "svs.h"
#include "soar_interface.h"
#include "image.h"
#include "visual_long_term_memory.h"
// #include "vision_operations.h"
#include "exact_visual_archetype.h"
#include "visual_working_memory.h"
#include "visual_wme.h"
// Base64 library for image transfer
#include "base64.h"

#ifdef ENABLE_ROS
const std::string visual_working_memory::ROS_TOPIC_NAME = "vwm";
#endif

////////////////
// CREATE VWM //
////////////////

visual_working_memory::visual_working_memory(svs* svsp, soar_interface* soar_int, Symbol* link) {
    _svs_ptr = svsp;
    si = soar_int;
    vwm_link = link;

    #ifdef ENABLE_ROS
    _svs_ptr->get_ros_interface()->create_rgb_publisher(ROS_TOPIC_NAME);
    #endif
}
visual_working_memory* visual_working_memory::clone(Symbol* link) {
    visual_working_memory* new_vwm = new visual_working_memory(_svs_ptr, si, link);

    typedef std::map<std::string, image_vwme>::iterator vwme_iterator;
    vwme_iterator curr_vwme = vwmes.begin();
    for (; curr_vwme != vwmes.end(); curr_vwme++) { 
        new_vwm->vwmes.emplace(curr_vwme->first, curr_vwme->second);
        new_vwm->metadata.emplace(curr_vwme->first, metadata[curr_vwme->first]);
    }
    return new_vwm;
}


////////////////////
// ADD A NEW VWME //
////////////////////
void visual_working_memory::_add_vwme(image_vwme new_vwme) {
    vwmes.emplace(new_vwme.get_id(), new_vwme);
    metadata.emplace(new_vwme.get_id(), vwme_metadata());
    _update();
}
void visual_working_memory::add_vwme(image_vwme new_vwme) {
    _add_vwme(new_vwme);
}
void visual_working_memory::add_image(opencv_image* new_image, std::string id) {
    image_vwme* new_vwme = new image_vwme(id, NULL);
    new_vwme->set_image(new_image->get_image());
    _add_vwme(*new_vwme);
}
// void visual_working_memory::add_varch(visual_archetype* new_varch) {
//     // TODO: Implement
// }


///////////////////
// REMOVE A VWME //
///////////////////
void visual_working_memory::remove_vwme(image_vwme target) {
    vwmes.erase(target.get_id());
    metadata.erase(target.get_id());
    _update();
}
void visual_working_memory::remove_vwme(std::string target) {
    vwmes.erase(target);
    metadata.erase(target);
    _update();
}


///////////////////////
// MANIPULATE A VWME //
///////////////////////
void visual_working_memory::translate_vwme(std::string vwme_ID, int dX, int dY) {
    int old_x = metadata[vwme_ID].x;
    int old_y = metadata[vwme_ID].y;

    int new_x = old_x + dX;
    int new_y = old_y + dY;

    move_vwme(vwme_ID, new_x, new_y);
    dirty = true;
    _update();
}
void visual_working_memory::move_vwme(std::string vwme_ID, int new_x, int new_y) {
    metadata[vwme_ID].x = new_x;
    metadata[vwme_ID].y = new_y;

    dirty = true;
    _update();
}
void visual_working_memory::rotate_vwme_rad(std::string vwme_ID, double rads) {
    float degs = rads * 180.0/M_PI;
    rotate_vwme_deg(vwme_ID, degs);
    dirty = true;
    _update();
}
void visual_working_memory::rotate_vwme_deg(std::string vwme_ID, double degs) {
    metadata[vwme_ID].rotation = degs;
    dirty = true;
    _update();
}
void visual_working_memory::flip_vwme_horiz(std::string vwme_ID) {
    metadata[vwme_ID].h_mirror = !metadata[vwme_ID].h_mirror;
    dirty = true;
    _update();
}
void visual_working_memory::flip_vwme_vert(std::string vwme_ID) {
    metadata[vwme_ID].v_mirror = !metadata[vwme_ID].v_mirror;
    dirty = true;
    _update();
}


//////////////
// DRAW VWM //
//////////////
void visual_working_memory::_generate_canvas() {
    typedef std::map<std::string, image_vwme>::iterator vwme_iterator;
    
    int left_bound = 0;
    int right_bound = 0;
    int top_bound = 0;
    int bottom_bound = 0;
    vwme_iterator vwme = vwmes.begin();
    for (; vwme != vwmes.end(); vwme++) {

        int vwme_width = vwme->second.get_image()->cols;
        int vwme_height = vwme->second.get_image()->rows;

        int vwme_x = metadata[vwme->first].x;
        int vwme_y = metadata[vwme->first].y;

        int vwme_left_bound = vwme_x - int(ceil(vwme_width/2.0));
        int vwme_right_bound = vwme_x + int(ceil(vwme_width/2.0));
        int vwme_top_bound = vwme_y - int(ceil(vwme_height/2.0));
        int vwme_bottom_bound = vwme_y + int(ceil(vwme_height/2.0));

        left_bound = MIN(left_bound, vwme_left_bound);
        right_bound = MAX(right_bound, vwme_right_bound);
        top_bound = MIN(top_bound, vwme_top_bound);
        bottom_bound = MAX(bottom_bound, vwme_bottom_bound);
    }

    int canvas_width = right_bound - left_bound;
    int canvas_height = bottom_bound - top_bound;
    canvas_width = MAX(canvas_width, 2);
    canvas_height = MAX(canvas_height, 2);

    canvas = cv::Mat::zeros(canvas_height, canvas_width, CV_8UC4);
    origin = cv::Point2i(-left_bound, -top_bound);
}

void visual_working_memory::_draw_vwme(image_vwme vwme, vwme_metadata mdata) {
    cv::Mat drawn_img = vwme.get_image()->clone();
    cv::Point2f vwme_center(drawn_img.cols/2.0, drawn_img.rows/2.0);

    cv::Mat rotation_mat = cv::getRotationMatrix2D(vwme_center, -1*mdata.rotation, 1.0);
    cv::warpAffine(drawn_img, drawn_img, rotation_mat, cv::Size(drawn_img.cols, drawn_img.rows));

    if (mdata.h_mirror) {
        cv::flip(drawn_img, drawn_img, 0);
    }
    if (mdata.v_mirror) {
        cv::flip(drawn_img, drawn_img, 1);
    }

    int left_bound = mdata.x - drawn_img.cols/2;
    int top_bound = mdata.y - drawn_img.rows/2;

    drawn_img.copyTo(canvas(cv::Rect(left_bound+origin.x, top_bound+origin.y, drawn_img.cols, drawn_img.rows)));
}

void visual_working_memory::_draw_canvas() {
    _generate_canvas();
    typedef std::map<std::string, image_vwme>::iterator vwme_iterator;
    
    vwme_iterator vwme = vwmes.begin();
    for (; vwme != vwmes.end(); vwme++) {
        _draw_vwme(vwme->second, metadata[vwme->first]);
    }
    dirty = false;
}

void visual_working_memory::_update() {
    _draw_canvas();
    #ifdef ENABLE_ROS
    _svs_ptr->get_ros_interface()->publish_rgb_image(ROS_TOPIC_NAME, canvas);
    #endif
}

opencv_image* visual_working_memory::get_percept() {
    opencv_image* percept_result = new opencv_image();
    percept_result->set_image(&canvas);
    return percept_result;
}

//////////////////
// CLI COMMANDS //
//////////////////
void visual_working_memory::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    c["save"] = new memfunc_proxy<visual_working_memory>(this, &visual_working_memory::cli_save);
    c["save"]->add_arg("FILEPATH", "The path of the file to save the image to.");

    c["recall"] = new memfunc_proxy<visual_working_memory>(this, &visual_working_memory::cli_recall);
    c["recall"]->add_arg("ID", "The id to retrieve the percept from.");
}

void visual_working_memory::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VISION INTERFACE ==========" << std::endl;
    os << "CLI USAGE:" << std::endl << std::endl;
    os << "svs <STATE> - Prints the last file uploaded to the agent, then this help text."<< std::endl;
    os << "svs <STATE>.save <FILEPATH> - Saves the current state of the agent's vision to the specified path."<< std::endl;
    os << "svs <STATE>.recall <ID> - Retrieves the specified archetype from visual memory and sets the visual input to the result." << std::endl;
    os << "======================================" << std::endl;
}

void visual_working_memory::cli_save(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Specify a file path to write to." << std::endl;
        return;
    }
    std::string filepath = args[0]; 

    opencv_image* percept = _svs_ptr->get_root_state()->get_vwm()->get_percept();
    cv::Mat image = *percept->get_image();

    cv::imwrite(filepath, image);
    os << "Wrote image to file " << filepath << std::endl;
}

void visual_working_memory::cli_recall(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "This command needs an ID parameter." << std::endl;
        return;
    }
    std::string name = args[0];
    opencv_image* percept = new opencv_image();
    _svs_ptr->get_v_mem_opencv()->recall(name, percept);
    _svs_ptr->image_callback(*percept->get_image());
    os << "Recalled percept " << name << std::endl;
    return;
}