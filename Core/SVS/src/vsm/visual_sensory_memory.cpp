#include <time.h>

#include "svs.h"
#include "visual_sensory_memory.h"
#include "image.h"
#include "symbol.h"
#include "soar_interface.h"
// Base64 library for image transfer
#include "base64.h"


#ifdef ENABLE_ROS
const std::string visual_sensory_memory::ROS_TOPIC_NAME_ = "vsm";
#endif

visual_sensory_memory::visual_sensory_memory(svs* svs_ptr, soar_interface* si, Symbol* vsm_link) {
    svs_ptr_ = svs_ptr;
    si_ = si;
    vsm_link_ = vsm_link;

    visual_buffer_ = new visual_buffer(si_, vsm_link);
    vop_graph_ = new visual_operation_graph(this, si_, vsm_link_);
    
    #ifdef ENABLE_ROS
    svs_ptr->get_ros_interface()->create_rgb_publisher(ROS_TOPIC_NAME_);
    #endif
}

visual_sensory_memory::~visual_sensory_memory() {
}


void visual_sensory_memory::update_visual_buffer(const cv::Mat& new_image) {
    visual_buffer_->add_new_frame(new_image);
    vop_graph_->evaluate();
}


/**
 * @brief Returns the visual percept as an opencv image. When called with
 * an argument, it retrieves the percept at the given index in the percept
 * buffer.
 * 
 */
opencv_image* visual_sensory_memory::get_vision() {
    return visual_buffer_->get_frame(0);
}
opencv_image* visual_sensory_memory::get_vision(int index) {
    return visual_buffer_->get_frame(index);
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

    c["inject"] =  new memfunc_proxy<visual_sensory_memory>(this, &visual_sensory_memory::cli_inject);
    c["inject"]->add_arg("IMGDATA", "Base64-encoded image data to inject.");
}

void visual_sensory_memory::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VSM INTERFACE ==========" << std::endl;
    os << "CURRENT TARGET FILE: " << target_filepath_ << std::endl;
    os << "CLI USAGE:" << std::endl << std::endl;
    os << "svs vsm - Prints the last file uploaded to the agent, then this help text."<< std::endl;
    os << "svs vsm.setfile <FILEPATH> - Sets the image upload target to the given filepath."<< std::endl;
    os << "svs vsm.load - Loads the current image upload target into the agent's vision."<< std::endl;
    os << "svs vsm.save <FILEPATH> - Saves the current state of the agent's vision to the specified path."<< std::endl;
    os << "svs vsm.inject <IMGDATA> - Injects the image defined by IMGDATA into VSM. Data should be base64."<< std::endl;
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

    target_filepath_ = new_target_filepath;
    return;
}

void visual_sensory_memory::cli_load(const std::vector<std::string>& args, std::ostream& os) {
    cv::Mat new_image = cv::imread(target_filepath_.c_str());
    printf("Loaded image: %d\n", (int)new_image.empty());
    svs_ptr_->image_callback(new_image);
    os << "Wrote image in " << target_filepath_ << " to visual input." << std::endl;
    return;
}

void visual_sensory_memory::cli_save(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Specify a file path to write to." << std::endl;
        return;
    }
    std::string filepath = args[0]; 

    opencv_image* percept = visual_buffer_->get_frame(0);
    cv::Mat image = *percept->get_image();

    cv::imwrite(filepath, image);
    os << "Wrote image to file " << filepath << std::endl;
}

void visual_sensory_memory::cli_inject(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Image data required." << std::endl;
        return;
    }

    std::string img_data = args[0];
    std::string decoded_data = base64_decode(img_data);
    std::vector<uchar> data(decoded_data.begin(), decoded_data.end());
    cv::Mat img = cv::imdecode(cv::Mat(data), -1);

    svs_ptr_->image_callback(img);
    os << "Injected image into visual input." << std::endl;
    return;
}

////////////////////////
// WME-BASED COMMANDS //
////////////////////////

void visual_sensory_memory::setfile(std::string filepath) {
    target_filepath_ = filepath;
}

void visual_sensory_memory::load() {
    cv::Mat new_image = cv::imread(target_filepath_.c_str(), cv::IMREAD_UNCHANGED);
    svs_ptr_->image_callback(new_image);
}

void visual_sensory_memory::save(std::string filepath) {
    visual_buffer_->get_frame(0)->draw_image(filepath);
}

bool visual_sensory_memory::_file_exists(std::string filepath) {
    FILE* exist_check = fopen(filepath.c_str(), "r");
    if (exist_check == NULL) {
        return false;
    }
    fclose(exist_check);
    return true;
}