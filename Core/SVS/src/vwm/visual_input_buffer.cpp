#include "visual_input_buffer.h"

/////////////////////////
// VISUAL BUFFER FRAME //
/////////////////////////

visual_buffer_frame::visual_buffer_frame(soar_interface* si, const cv::Mat& img, wme* frame_link)
    : si(si), frame_wme(frame_link)
{
    index = 0;
    image = new opencv_image();
    image->update_image(img);
    timestamp_time_t = std::time(NULL);
    timestamp = (int)timestamp_time_t;
    frame_sym = si->get_wme_val(frame_wme);

    timestamp_sym = si->make_sym(timestamp);
    timestamp_wme = si->make_wme(frame_sym, std::string("timestamp"), timestamp_sym);

    index_sym = si->make_sym(index);
    index_wme = si->make_wme(frame_sym, std::string("index"), index_sym);
}

visual_buffer_frame::~visual_buffer_frame()
{
    delete image;
    si->remove_wme(timestamp_wme);
    si->remove_wme(index_wme);
    si->remove_wme(frame_wme);
}

void visual_buffer_frame::increment_index() {
    index++;
    si->remove_wme(index_wme);
    index_sym = si->make_sym(index);
    index_wme = si->make_wme(frame_sym, std::string("index"), index_sym);
}


/////////////////////////
// VISUAL INPUT BUFFER //
/////////////////////////

visual_input_buffer::visual_input_buffer(soar_interface* si, Symbol* vsm_link)
    : si(si) {
        buffer = new visual_buffer_frame*[MAX_SIZE];
        for (int i=0; i<MAX_SIZE; i++) {
            buffer[i] = NULL;
        }
        size = 0;

        visual_buffer_link = si->get_wme_val(si->make_id_wme(vsm_link, "visual-buffer"));

        size_sym = si->make_sym(size);
        size_wme = si->make_wme(visual_buffer_link, std::string("size"), size_sym);

        newest_update_sym = si->make_sym(newest_update);
        newest_update_wme = si->make_wme(visual_buffer_link, std::string("newest-update"), newest_update_sym);

        oldest_update_sym = si->make_sym(oldest_update);
        oldest_update_wme =  si->make_wme(visual_buffer_link, std::string("oldest-update"), oldest_update_sym);

        frames_link = si->get_wme_val(si->make_id_wme(visual_buffer_link, std::string("frames")));
}

visual_input_buffer::~visual_input_buffer() {
    for (int i=0; i<size; i++) {
        delete buffer[i];
    }
    delete buffer;

    si->remove_wme(size_wme);
    si->remove_wme(newest_update_wme);
    si->remove_wme(oldest_update_wme);
    si->del_sym(frames_link);
}

bool visual_input_buffer::add_new_frame(const cv::Mat& new_image) {
    if (size != MAX_SIZE) {
        size ++;
        si->remove_wme(size_wme);
        size_sym = si->make_sym(size);
        size_wme = si->make_wme(visual_buffer_link, std::string("size"), size_sym);
    }

    // Eject the last element in the percept buffer
    if (buffer[MAX_SIZE-1] != NULL) {
        delete buffer[MAX_SIZE-1];
        buffer[MAX_SIZE-1] = NULL;

    }

    // Move the remaining elements up
    for (int i=MAX_SIZE-2; i>=0; i--) {
        if (buffer[i] == NULL) { continue; }
        buffer[i+1] = buffer[i];
        buffer[i+1]->increment_index();
    }

    // Create the latest element
    wme* frame_link = si->make_id_wme(frames_link, std::string("frame"));
    buffer[0] = new visual_buffer_frame(si, new_image, frame_link);

    // Update the "updated" values
    newest_update = buffer[0]->get_timestamp();
    si->remove_wme(newest_update_wme);
    newest_update_sym = si->make_sym(newest_update);
    newest_update_wme = si->make_wme(visual_buffer_link, std::string("newest-update"), newest_update_sym);

    oldest_update = buffer[size-1]->get_timestamp();
    si->remove_wme(oldest_update_wme);
    oldest_update_sym = si->make_sym(oldest_update);
    oldest_update_wme = si->make_wme(visual_buffer_link, std::string("oldest-update"), oldest_update_sym);
    return true;
}

opencv_image* visual_input_buffer::get_frame() {
    return buffer[0]->get_image();
}

opencv_image* visual_input_buffer::get_frame(int index) {
    if (index<0 || index >= size) {
        return NULL;
    }
    return buffer[index]->get_image();
}


//////////////////
// FILEPATH VIB //
//////////////////

filepath_visual_input_buffer::filepath_visual_input_buffer(soar_interface* si, Symbol* vsm_link, std::string fp)
    : visual_input_buffer(si, vsm_link)
{
    if (file_exists(fp)) {
        filepath = fp;
    } else {
        throw std::runtime_error("Filepath does not exist");
    }
}

filepath_visual_input_buffer::~filepath_visual_input_buffer() {
    delete &filepath;
}

bool filepath_visual_input_buffer::file_exists(std::string fp) {
    FILE* exist_check = fopen(filepath.c_str(), "r");
    if (exist_check == NULL) {
        return false;
    }
    fclose(exist_check);
    return true;
}

// CLIPROXY METHODS
///////////////////
void filepath_visual_input_buffer::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    c["setfile"] = new memfunc_proxy<visual_working_memory>(this, &visual_working_memory::cli_setfile);
    c["setfile"]->add_arg("FILEPATH", "The path of the file to load the image from.");

    c["load"] = new memfunc_proxy<visual_working_memory>(this, &visual_working_memory::cli_load);

    c["save"] = new memfunc_proxy<visual_working_memory>(this, &visual_working_memory::cli_save);
    c["save"]->add_arg("FILEPATH", "The path of the file to save the image to.");

    c["inject"] =  new memfunc_proxy<visual_working_memory>(this, &visual_working_memory::cli_inject);
    c["inject"]->add_arg("IMGDATA", "Base64-encoded image data to inject.");
}

void filepath_visual_input_buffer::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== FILE VISUAL INPUT BUFFER INTERFACE ==========" << std::endl;
    os << "CURRENT TARGET FILE: " << filepath << std::endl;
    os << "CLI USAGE:" << std::endl << std::endl;
    os << "svs vsm - Prints the last file uploaded to the agent, then this help text."<< std::endl;
    os << "svs vsm.setfile <FILEPATH> - Sets the image upload target to the given filepath."<< std::endl;
    os << "svs vsm.load - Loads the current image upload target into the agent's vision."<< std::endl;
    os << "svs vsm.inject <IMGDATA> - Injects the image defined by IMGDATA into VSM. Data should be base64."<< std::endl;
    os << "========================================================" << std::endl;
}

// CLI-BASED COMMANDS
/////////////////////

void filepath_visual_input_buffer::cli_setfile(const std::vector<std::string>& args, std::ostream& os) {
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

void filepath_visual_input_buffer::cli_load(const std::vector<std::string>& args, std::ostream& os) {
    cv::Mat new_image = cv::imread(target_filepath_, cv::IMREAD_UNCHANGED);
    printf("Loaded image: %s\n", new_image.empty() ? "FALSE" : "TRUE");
    svs_ptr->image_callback(new_image);
    os << "Wrote image in " << target_filepath_ << " to visual input." << std::endl;
    return;
}

void filepath_visual_input_buffer::cli_inject(const std::vector<std::string>& args, std::ostream& os) {
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

// WME-BASED COMMANDS
/////////////////////

void filepath_visual_input_buffer::setfile(std::string filepath) {
    target_filepath_ = filepath;
}

void filepath_visual_input_buffer::load() {
    cv::Mat new_image = cv::imread(filepath.c_str(), cv::IMREAD_UNCHANGED);
    add_new_frame(new_image);
}
