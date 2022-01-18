#include <time.h>

#include "svs.h"
#include "visual_sensory_memory.h"
#include "image.h"
#include "symbol.h"
#include "soar_interface.h"
// Base64 library for image transfer
#include "base64.h"


#ifdef ENABLE_ROS
const std::string visual_sensory_memory::ROS_TOPIC_NAME = "vsm";
#endif

visual_sensory_memory::visual_sensory_memory(svs* _svs_ptr, soar_interface* _si) {
    svs_ptr = _svs_ptr;
    si = _si;
    update_counter = 0;

    vop_graph = new visual_operation_graph(this);

    // Fill the percept buffer with NULLs 
    for (int i=0; i<PERCEPT_BUFFER_SIZE; i++) {
        percept_buffer[i] = NULL;
    }
    
    #ifdef ENABLE_ROS
    svs_ptr->get_ros_interface()->create_rgb_publisher(ROS_TOPIC_NAME);
    #endif
}

visual_sensory_memory::~visual_sensory_memory() {
}

/**
 * @brief Called at the beginning of every output phase. Updates the VSM links
 * so they reflect the VOG structure.
 * 
 */
void visual_sensory_memory::output_callback() {
    // std::vector<Symbol *>::iterator vsm_link_itr;
    // for (vsm_link_itr=vsm_links.begin(); vsm_link_itr!=vsm_links.end(); vsm_link_itr++) {

    // }
}

void visual_sensory_memory::add_wm_link(Symbol* _vsm_link) {
    vsm_links.push_back(_vsm_link);
    si->make_wme(_vsm_link, std::string("updated"), si->make_sym(update_counter));
    if (vsm_links.size() == 1) {
        vops_link = si->make_id_wme(_vsm_link, "vops");
    }
}

void visual_sensory_memory::update_percept_buffer(const cv::Mat& new_image) {
    printf("Updating percept buffer... ");
    // Eject the last element in the percept buffer
    if (percept_buffer[PERCEPT_BUFFER_SIZE-1] != NULL) {
        delete percept_buffer[PERCEPT_BUFFER_SIZE-1];
        percept_buffer[PERCEPT_BUFFER_SIZE-1] = NULL;
    }

    // Move the remaining elements up
    for (int i=PERCEPT_BUFFER_SIZE-2; i>=0; i--) {
        percept_buffer[i+1] = percept_buffer[i];
    } 

    // Create the latest element
    percept_buffer[0] = new opencv_image();
    percept_buffer[0]->update_image(new_image);
    update_counter++;

    std::vector<Symbol*>::iterator vsm_wme_itr;
    for (vsm_wme_itr=vsm_links.begin(); vsm_wme_itr!=vsm_links.end(); vsm_wme_itr++) {
        _update_wm_link(*vsm_wme_itr);
    }
    vop_graph->evaluate();
    // draw_percept_buffer();
}

void visual_sensory_memory::_update_wm_link(Symbol* vsm_wme) {
    printf("updating WM link... ");
    wme_vector vsm_wme_children;
    si->get_child_wmes(vsm_wme, vsm_wme_children);

    wme_vector::iterator child_itr;
    for (child_itr=vsm_wme_children.begin(); child_itr!=vsm_wme_children.end(); child_itr++) {
        wme* child_wme = *child_itr;
        char* child_attr = child_wme->attr->to_string();
        printf("checking child %s... ", child_attr);

        if (strcmp(child_attr, "updated") == 0) {
            si->remove_wme(child_wme);
            si->make_wme(vsm_wme, std::string("updated"), si->make_sym(update_counter));
            printf("updated %d\n", update_counter);
            continue;
        }
    }
}


void visual_sensory_memory::draw_percept_buffer() {
    percept_buffer[0]->draw_image("percept_buffer.png");

    #ifdef ENABLE_ROS
    svs_ptr->get_ros_interface()->publish_rgb_image(ROS_TOPIC_NAME, *percept_buffer[0]->get_image());
    #endif
}


opencv_image* visual_sensory_memory::get_vision() {
    return percept_buffer[0];
}


opencv_image* visual_sensory_memory::get_vision(int index) {
    if (index >= PERCEPT_BUFFER_SIZE) {
        return NULL;
    }
    return percept_buffer[index];
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
    os << "CURRENT TARGET FILE: " << _target_filepath << std::endl;
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

void visual_sensory_memory::cli_inject(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Image data required." << std::endl;
        return;
    }

    std::string img_data = args[0];
    std::string decoded_data = base64_decode(img_data);
    std::vector<uchar> data(decoded_data.begin(), decoded_data.end());
    cv::Mat img = cv::imdecode(cv::Mat(data), -1);

    svs_ptr->image_callback(img);
    os << "Injected image into visual input." << std::endl;
    return;
}

////////////////////////
// WME-BASED COMMANDS //
////////////////////////

void visual_sensory_memory::setfile(std::string filepath) {
    _target_filepath = filepath;
}

void visual_sensory_memory::load() {
    cv::Mat new_image = cv::imread(_target_filepath.c_str(), cv::IMREAD_UNCHANGED);
    svs_ptr->image_callback(new_image);
}

void visual_sensory_memory::save(std::string filepath) {
    percept_buffer[0]->draw_image(filepath);
}

bool visual_sensory_memory::_file_exists(std::string filepath) {
    FILE* exist_check = fopen(filepath.c_str(), "r");
    if (exist_check == NULL) {
        return false;
    }
    fclose(exist_check);
    return true;
}