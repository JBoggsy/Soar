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
#include "vision_interface.h"

vision_interface::vision_interface(svs* svs_ptr) {
    _svs_ptr = svs_ptr;
    _target_filepath = std::string("");
}
vision_interface::~vision_interface() {}

void vision_interface::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    c["setfile"] = new memfunc_proxy<vision_interface>(this, &vision_interface::setfile);
    c["setfile"]->add_arg("FILEPATH", "The path of the file to load the image from.");

    c["load"] = new memfunc_proxy<vision_interface>(this, &vision_interface::load);
    
    c["save"] = new memfunc_proxy<vision_interface>(this, &vision_interface::save);
    c["save"]->add_arg("FILEPATH", "The path of the file to save the image to.");
}

void vision_interface::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VISION INTERFACE ==========" << std::endl;
    os << "CURRENT TARGET FILE: " << _target_filepath << std::endl;
    os << "CLI USAGE:" << std::endl << std::endl;
    os << "svs vision - Prints the last file uploaded to the agent, then this help text."<< std::endl;
    os << "svs vision.setfile <FILEPATH> - Sets the image upload target to the given filepath."<< std::endl;
    os << "svs vision.load - Loaads the current image upload target into the agent's vision."<< std::endl;
    os << "svs vision.save <FILEPATH> - Saves the current state of the agent's vision to the specified path."<< std::endl;
    os << "======================================" << std::endl;
}

void vision_interface::setfile(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Specify a file path to load." << std::endl;
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

void vision_interface::load(const std::vector<std::string>& args, std::ostream& os) {
    cv::Mat new_image = cv::imread(_target_filepath.c_str());
    _svs_ptr->image_callback(new_image);
    os << "Wrote image in " << _target_filepath << " to visual input." << std::endl;
    return;
}

void vision_interface::save(const std::vector<std::string>& args, std::ostream& os) {
    os << "Saving visual input state is not implemented yet." << std::endl;
    return;
}

bool vision_interface::_file_exists(std::string filepath) {
    FILE* exist_check = fopen(filepath.c_str(), "r");
    if (exist_check == NULL) {
        return false;
    }
    fclose(exist_check);
    return true;
}