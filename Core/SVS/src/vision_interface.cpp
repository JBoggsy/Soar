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
#include "visual_long_term_memory.h"
#include "vision_operations.h"
#include "vision_interface.h"
#include "exact_visual_archetype.h"

vision_interface::vision_interface(svs* svs_ptr) {
    _svs_ptr = svs_ptr;
    _target_filepath = std::string("");
}
vision_interface::~vision_interface() {}

void vision_interface::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    c["setfile"] = new memfunc_proxy<vision_interface>(this, &vision_interface::cli_setfile);
    c["setfile"]->add_arg("FILEPATH", "The path of the file to load the image from.");

    c["load"] = new memfunc_proxy<vision_interface>(this, &vision_interface::cli_load);
    
    c["save"] = new memfunc_proxy<vision_interface>(this, &vision_interface::cli_save);
    c["save"]->add_arg("FILEPATH", "The path of the file to save the image to.");

    c["remember"] = new memfunc_proxy<vision_interface>(this, &vision_interface::cli_remember);
    c["remember"]->add_arg("ID", "The id to store the percept in.");

    c["recall"] = new memfunc_proxy<vision_interface>(this, &vision_interface::cli_recall);
    c["recall"]->add_arg("ID", "The id to retrieve the percept from.");

    c["match"] = new memfunc_proxy<vision_interface>(this, &vision_interface::cli_match);

    c["rotate"] = new memfunc_proxy<vision_interface>(this, &vision_interface::cli_rotate);
    c["rotate"]->add_arg("ANGLE", "The amount to rotate the percept, one of 90, 180, or 270.");

    c["export_imagination"] = new memfunc_proxy<vision_interface>(this, &vision_interface::cli_export_imagination);
    c["export_imagination"]->add_arg("FILEPATH", "The path of the file to save the image to.");
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
    os << "svs vision.rotate <ANGLE> - Rotates the current percept by 90, 180, or 270 degrees clockwise." << std::endl;
    os << "svs vision.export_imagination <FILEPATH> - Export the currentstate of the root svs state imagination to the given file." << std::endl;
    os << "======================================" << std::endl;
}

//////////////////////
// VISION COMMANDS //
////////////////////

void vision_interface::setfile(std::string filepath) {
    _target_filepath = filepath;
}

void vision_interface::load() {
    cv::Mat new_image = cv::imread(_target_filepath.c_str());
    _svs_ptr->image_callback(new_image);
}

void vision_interface::save(std::string filepath) {
    opencv_image* percept = _svs_ptr->get_root_state()->get_image_opencv();
    cv::Mat image = *percept->get_image();
    cv::imwrite(filepath, image);
}

void vision_interface::remember(std::string ID) {
    opencv_image* percept = _svs_ptr->get_root_state()->get_image_opencv();
    _svs_ptr->get_v_mem_opencv()->store_percept(percept, ID);
}

void vision_interface::recall(std::string ID) {
    opencv_image* percept = new opencv_image();
    _svs_ptr->get_v_mem_opencv()->recall(ID, percept);
    _svs_ptr->image_callback(*percept->get_image());
}

void vision_interface::match(vmem_match* output) {
    opencv_image* percept = _svs_ptr->get_root_state()->get_image_opencv();
    _svs_ptr->get_v_mem_opencv()->match(percept, output);
    printf("Match found: %s, %f\n", output->entity_id.c_str(), output->confidence);
}

void vision_interface::rotate(int amount) {
    opencv_image* percept = _svs_ptr->get_root_state()->get_image_opencv();
    switch (amount) {
        case 90:
            rotate_90(*percept);
            break;
        case 180:
            rotate_180(*percept);
            break;
        case 270:
            rotate_270(*percept);
            break;
    }
}

///////////////////
// CLI COMMANDS //
/////////////////

void vision_interface::cli_setfile(const std::vector<std::string>& args, std::ostream& os) {
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

void vision_interface::cli_load(const std::vector<std::string>& args, std::ostream& os) {
    cv::Mat new_image = cv::imread(_target_filepath.c_str());
    printf("Loaded image: %d\n", (int)new_image.empty());
    _svs_ptr->image_callback(new_image);
    os << "Wrote image in " << _target_filepath << " to visual input." << std::endl;
    return;
}

void vision_interface::cli_save(const std::vector<std::string>& args, std::ostream& os) {
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

void vision_interface::cli_remember(const std::vector<std::string>& args, std::ostream& os) {
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

void vision_interface::cli_recall(const std::vector<std::string>& args, std::ostream& os) {
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

void vision_interface::cli_match(const std::vector<std::string>& args, std::ostream& os) {
    opencv_image* percept = _svs_ptr->get_root_state()->get_image_opencv();
    vmem_match match;
    _svs_ptr->get_v_mem_opencv()->match(percept, &match);
    std::string match_id = match.entity_id;
    float match_confidence = match.confidence;

    os << "Matched the current percept to " << match_id << " with " << match_confidence << " conf." << std::endl;    
    return;
}

void vision_interface::cli_rotate(const std::vector<std::string>& args, std::ostream& os) {
    opencv_image* percept = _svs_ptr->get_root_state()->get_image_opencv();
    int rotation_amount = std::stoi(args[0]);
    switch (rotation_amount) {
        case 90:
            rotate_90(*percept);
            break;
        case 180:
            rotate_180(*percept);
            break;
        case 270:
            rotate_270(*percept);
            break;
    }

    os << "Rotated the current percept by " << args[0] << std::endl;    
    return;
}

void vision_interface::cli_export_imagination(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Specify a file path to write to." << std::endl;
        return;
    }
    std::string filepath = args[0]; 

    opencv_image imagined_percept = opencv_image();
    _svs_ptr->get_root_state()->get_imagination()->get_image(imagined_percept);
    cv::Mat* image = imagined_percept.get_image();

    cv::imwrite(filepath, *image);
    os << "Wrote image to file " << filepath << std::endl;
}

bool vision_interface::_file_exists(std::string filepath) {
    FILE* exist_check = fopen(filepath.c_str(), "r");
    if (exist_check == NULL) {
        return false;
    }
    fclose(exist_check);
    return true;
}