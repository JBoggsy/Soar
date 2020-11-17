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
#include "image.h"
#include "visual_memory.h"
#include "vision_interface.h"
#include "exact_visual_archetype.h"

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

    c["remember"] = new memfunc_proxy<vision_interface>(this, &vision_interface::remember);
    c["remember"]->add_arg("ID", "The id to store the percept in.");

    c["recall"] = new memfunc_proxy<vision_interface>(this, &vision_interface::recall);
    c["recall"]->add_arg("ID", "The id to retrieve the percept from.");

    c["match"] = new memfunc_proxy<vision_interface>(this, &vision_interface::match);
}

void vision_interface::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VISION INTERFACE ==========" << std::endl;
    os << "CURRENT TARGET FILE: " << _target_filepath << std::endl;
    os << "CLI USAGE:" << std::endl << std::endl;
    os << "svs vision - Prints the last file uploaded to the agent, then this help text."<< std::endl;
    os << "svs vision.setfile <FILEPATH> - Sets the image upload target to the given filepath."<< std::endl;
    os << "svs vision.load - Loads the current image upload target into the agent's vision."<< std::endl;
    os << "svs vision.save <FILEPATH> - Saves the current state of the agent's vision to the specified path."<< std::endl;
    os << "svs vision.remember <ID> - Adds the current visual input to visual memory." << std::endl;
    os << "svs vision.recall <ID> - Retrieves the specified archetype from visual memory and sets the visual input to the result." << std::endl;
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
    printf("Loaded image: %d\n", (int)new_image.empty());
    _svs_ptr->image_callback(new_image);
    os << "Wrote image in " << _target_filepath << " to visual input." << std::endl;
    return;
}

void vision_interface::save(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Specify a file path to write to." << std::endl;
        return;
    }
    std::string filepath = args[0]; 

    opencv_image* percept = _svs_ptr->get_root_state()->get_image_opencv();
    cv::Mat image = *percept->get_image();

    cv::imwrite(filepath, image);
    os << "Wrote image to file " << filepath << std::endl;
}

void vision_interface::remember(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "This command needs an ID parameter." << std::endl;
        return;
    }
    std::string name = args[0];

    opencv_image* percept = _svs_ptr->get_root_state()->get_image_opencv();
    _svs_ptr->get_v_mem_opencv()->store_percept(percept, name);
    os << "Remembered current percept as " << name << std::endl;
    return;
}

void vision_interface::recall(const std::vector<std::string>& args, std::ostream& os) {
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

void vision_interface::match(const std::vector<std::string>& args, std::ostream& os) {
    opencv_image* percept = _svs_ptr->get_root_state()->get_image_opencv();
    vmem_match match = _svs_ptr->get_v_mem_opencv()->match(percept);
    std::string match_id = match.entity_id;
    float match_confidence = match.confidence;

    os << "Matched the current percept to " << match_id << " with " << match_confidence << " conf." << std::endl;    
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