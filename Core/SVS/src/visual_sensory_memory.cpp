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

//////////////////////
// CLIPROXY METHODS //
//////////////////////
void visual_sensory_memory::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    c["setfile"] = new memfunc_proxy<visual_sensory_memory>(this, &visual_sensory_memory::cli_setfile);
    c["setfile"]->add_arg("FILEPATH", "The path of the file to load the image from.");

    c["load"] = new memfunc_proxy<visual_sensory_memory>(this, &visual_sensory_memory::cli_load);
    
    c["save"] = new memfunc_proxy<visual_sensory_memory>(this, &visual_sensory_memory::cli_save);
    c["save"]->add_arg("FILEPATH", "The path of the file to save the image to.");
}

void visual_sensory_memory::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VSM INTERFACE ==========" << std::endl;
    os << "CURRENT TARGET FILE: " << _target_filepath << std::endl;
    os << "CLI USAGE:" << std::endl << std::endl;
    os << "svs vsm - Prints the last file uploaded to the agent, then this help text."<< std::endl;
    os << "svs vsm.setfile <FILEPATH> - Sets the image upload target to the given filepath."<< std::endl;
    os << "svs vsm.load - Loads the current image upload target into the agent's vision."<< std::endl;
    os << "svs vsm.save <FILEPATH> - Saves the current state of the agent's vision to the specified path."<< std::endl;
    os << "======================================" << std::endl;
}

////////////////////////
// CLI-BASED COMMANDS //
////////////////////////

void visual_sensory_memory::cli_setfile(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "No filepath specified." << std::endl;
        return;
    }

    // Ensure the specified file exists
    std::string new_target_filepath(args[0]);
    if (!_file_exists(new_target_filepath)) {
        os << "Specified path " << new_target_filepath <<" does not exist." << std::endl;
        return;
    }

    _target_filepath = new_target_filepath;
    return;
}

void visual_sensory_memory::cli_load(const std::vector<std::string>& args, std::ostream& os) {
    cv::Mat new_image = cv::imread(_target_filepath.c_str());
    printf("Loaded image: %d\n", (int)new_image.empty());
    svs_ptr->image_callback(new_image);
    os << "Wrote image in " << _target_filepath << " to visual input." << std::endl;
    return;
}

void visual_sensory_memory::cli_save(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Specify a file path to write to." << std::endl;
        return;
    }
    std::string filepath = args[0]; 

    opencv_image* percept = percept_buffer[0];
    cv::Mat image = *percept->get_image();

    cv::imwrite(filepath, image);
    os << "Wrote image to file " << filepath << std::endl;
}

////////////////////////
// WME-BASED COMMANDS //
////////////////////////

void visual_sensory_memory::setfile(std::string filepath) {
    _target_filepath = filepath;
}

void visual_sensory_memory::load() {
    cv::Mat new_image = cv::imread(_target_filepath.c_str());
    svs_ptr->image_callback(new_image);
}

void visual_sensory_memory::save(std::string filepath) {
    percept_buffer[0]->draw_image(filepath);
}

opencv_image* visual_sensory_memory::give_vision() {
    return percept_buffer[0];
}


bool visual_sensory_memory::_file_exists(std::string filepath) {
    FILE* exist_check = fopen(filepath.c_str(), "r");
    if (exist_check == NULL) {
        return false;
    }
    fclose(exist_check);
    return true;
}