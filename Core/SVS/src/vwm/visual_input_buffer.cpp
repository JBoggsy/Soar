#include "visual_input_buffer.h"
#include "base64.h"


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

visual_input_buffer::visual_input_buffer(soar_interface* si, Symbol* vib_manager_link, std::string vib_id)
    : si(si), vib_id(vib_id) {
        buffer = new visual_buffer_frame*[MAX_SIZE];
        for (int i=0; i<MAX_SIZE; i++) {
            buffer[i] = NULL;
        }
        size = 0;

        visual_buffer_link = si->get_wme_val(si->make_id_wme(vib_manager_link, "visual-buffer"));

        vib_id_sym = si->make_sym(vib_id);
        vib_id_wme = si->make_wme(visual_buffer_link, std::string("vib-id"), vib_id_sym);

        size_sym = si->make_sym(size);
        size_wme = si->make_wme(visual_buffer_link, std::string("size"), size_sym);

        newest_update_sym = si->make_sym(newest_update);
        newest_update_wme = si->make_wme(visual_buffer_link, std::string("newest-update"), newest_update_sym);

        oldest_update_sym = si->make_sym(oldest_update);
        oldest_update_wme =  si->make_wme(visual_buffer_link, std::string("oldest-update"), oldest_update_sym);

        frames_link = si->get_wme_val(si->make_id_wme(visual_buffer_link, std::string("frames")));

        type = generic;
}

visual_input_buffer::~visual_input_buffer() {
    for (int i=0; i<size; i++) {
        delete buffer[i];
    }
    delete buffer;
    delete &vib_id;

    si->remove_wme(vib_id_wme);
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

void visual_input_buffer::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    c["inject"] =  new memfunc_proxy<visual_input_buffer>(this, &visual_input_buffer::cli_inject);
    c["inject"]->add_arg("IMGDATA", "Base64-encoded image data to inject.");
}

void visual_input_buffer::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VISUAL INPUT BUFFER INTERFACE ==========" << std::endl;
    os << "VISUAL INPUT BUFFER ID: " << vib_id << std::endl;
    os << "svs vibmgr." << vib_id << ".inject <IMGDATA> - Injects the image defined by IMGDATA into VSM. Data should be base64."<< std::endl;
    os << "========================================================" << std::endl;
}

void visual_input_buffer::cli_inject(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "Image data required." << std::endl;
        return;
    }

    std::string img_data = args[0];
    std::string decoded_data = base64_decode(img_data);
    std::vector<uchar> data(decoded_data.begin(), decoded_data.end());
    cv::Mat img = cv::imdecode(cv::Mat(data), -1);

    if (add_new_frame(img)) {
        os << "Injected image into visual input." << std::endl;
    } else {
        os << "Failed to inject image into visual input." << std::endl;
    }

    return;
}


//////////////////
// FILEPATH VIB //
//////////////////

filepath_visual_input_buffer::filepath_visual_input_buffer(soar_interface* si, Symbol* vib_manager_link, std::string vib_id, std::string fp)
    : visual_input_buffer(si, vib_manager_link, vib_id)
{
    if (file_exists(fp)) {
        filepath = fp;
    } else {
        throw std::runtime_error("Filepath does not exist");
    }

    type = vib_type::filepath;
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

void filepath_visual_input_buffer::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    c["setfile"] = new memfunc_proxy<filepath_visual_input_buffer>(this, &filepath_visual_input_buffer::cli_setfile);
    c["setfile"]->add_arg("FILEPATH", "The path of the file to load the image from.");

    c["load"] = new memfunc_proxy<filepath_visual_input_buffer>(this, &filepath_visual_input_buffer::cli_load);

    c["inject"] =  new memfunc_proxy<filepath_visual_input_buffer>(this, &filepath_visual_input_buffer::cli_inject);
    c["inject"]->add_arg("IMGDATA", "Base64-encoded image data to inject.");
}

void filepath_visual_input_buffer::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== FILE VISUAL INPUT BUFFER INTERFACE ==========" << std::endl;
    os << "CURRENT TARGET FILE: " << filepath << std::endl;
    os << "CLI USAGE:" << std::endl << std::endl;
    os << "svs vibmgr." << vib_id << " - Prints the last file uploaded to the agent, then this help text."<< std::endl;
    os << "svs vibmgr." << vib_id << ".setfile <FILEPATH> - Sets the image upload target to the given filepath."<< std::endl;
    os << "svs vibmgr." << vib_id << ".load - Loads the current image upload target into the agent's vision."<< std::endl;
    os << "========================================================" << std::endl;
}

void filepath_visual_input_buffer::cli_setfile(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "No filepath specified." << std::endl;
        return;
    }

    // Ensure the specified file exists
    std::string new_target_filepath(args[0]);
    if (!file_exists(new_target_filepath)) {
        os << "Specified path " << new_target_filepath <<" does not exist." << std::endl;
        return;
    }

    filepath = new_target_filepath;
    return;
}

void filepath_visual_input_buffer::cli_load(const std::vector<std::string>& args, std::ostream& os) {
    cv::Mat new_image = cv::imread(filepath, cv::IMREAD_UNCHANGED);
    printf("Loaded image: %s\n", new_image.empty() ? "FALSE" : "TRUE");
    if (add_new_frame(new_image)) {
        os << "Wrote image in " << filepath << " to visual input." << std::endl;
    } else {
        os << "Failed to write image in " << filepath << " to visual input." << std::endl;
    }

    return;
}

void filepath_visual_input_buffer::load() {
    cv::Mat new_image = cv::imread(filepath.c_str(), cv::IMREAD_UNCHANGED);
    add_new_frame(new_image);
}


/////////////////////////////////
// VISUAL INPUT BUFFER MANAGER //
/////////////////////////////////

visual_input_buffer_manager::visual_input_buffer_manager(soar_interface* si, Symbol* vib_link)
    : si(si), svs_link(svs_link), vib_manager_link(vib_link) {
    num_vibs_wme = si->make_wme(vib_manager_link, "num-vibs", (int)visual_input_buffers.size());
}

visual_input_buffer_manager::~visual_input_buffer_manager() {
    vib_map::iterator vibs_itr;
    for (vibs_itr=visual_input_buffers.begin(); vibs_itr!=visual_input_buffers.end(); vibs_itr++) {
        del_visual_input_buffer(vibs_itr->first);
    }
}

void visual_input_buffer_manager::add_visual_input_buffer(std::string vib_id, visual_input_buffer* new_vib) {
    visual_input_buffers[vib_id] = new_vib;
    // TODO: Add listener?
    si->remove_wme(num_vibs_wme);
    num_vibs_wme = si->make_wme(vib_manager_link, "num-vibs", get_num_vibs());
}

void visual_input_buffer_manager::del_visual_input_buffer(std::string tgt_vib_id){
    vib_map::iterator vib_itr = visual_input_buffers.find(tgt_vib_id);
    if (vib_itr != visual_input_buffers.end()) {
        visual_input_buffers.erase(vib_itr);
        si->remove_wme(num_vibs_wme);
        num_vibs_wme = si->make_wme(vib_manager_link, "num-vibs", get_num_vibs());
    }
}

void visual_input_buffer_manager::proxy_get_children(std::map<std::string, cliproxy*>& c) {
    c["add-vib"] = new memfunc_proxy<visual_input_buffer_manager>(this, &visual_input_buffer_manager::cli_add_visual_input_buffer);
    c["add-vib"]->add_arg("NAME", "A unique name to associate the VIB with.");

    c["add-file-vib"] = new memfunc_proxy<visual_input_buffer_manager>(this, &visual_input_buffer_manager::cli_add_filepath_visual_input_buffer);
    c["add-file-vib"]->add_arg("FILEPATH", "The path of the file to load the image from.");
    c["add-file-vib"]->add_arg("NAME", "A unique name to associate the VIB with.");

    c["del-vib"] = new memfunc_proxy<visual_input_buffer_manager>(this, &visual_input_buffer_manager::cli_del_visual_input_buffer);
    c["del-vib"]->add_arg("NAME", "The unique name of the VIB to be deleted.");

    c["get-vibs"] = new memfunc_proxy<visual_input_buffer_manager>(this, &visual_input_buffer_manager::cli_get_visual_input_buffers);

    vib_map::iterator vib_itr = visual_input_buffers.begin();
    for (; vib_itr != visual_input_buffers.end(); vib_itr++) {
        c[vib_itr->first] = vib_itr->second;
    }
}

void visual_input_buffer_manager::proxy_use_sub(const std::vector<std::string>& args, std::ostream& os) {
    os << "========== VIB MANAGER INTERFACE ==========" << std::endl;
    os << "CURRENT NUMBER OF VIBS: " << get_num_vibs() << std::endl;
    os << "CLI USAGE:" << std::endl << std::endl;
    os << "svs vibmgr - Prints the number of visual input buffers, then this help text."<< std::endl;
    os << "svs vibmgr.add-vib <NAME> - Adds a generic (injection-only) visual input buffer with the given name."<< std::endl;
    os << "svs vibmgr.add-file-vib <FILEPATH> <NAME> - Adds a file-based visual input buffer targeting the specified file."<< std::endl;
    os << "svs vibmgr.del-vib <NAME> - Removes the visual input buffer with the given name."<< std::endl;
    os << "svs vibmgr.get-vibs - Lists the names and types of the current visual input buffers." << std::endl;
    os << "svs vibmgr.<vib-name> - Accesses the CLI interface of the specified visual input buffer." << std::endl;
    os << "========================================================" << std::endl;
}

void visual_input_buffer_manager::cli_add_visual_input_buffer(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "No vib name specified." << std::endl;
        return;
    }

    std::string new_vib_id(args[0]);
    add_visual_input_buffer(new_vib_id, new visual_input_buffer(si, vib_manager_link, new_vib_id));

    os << "Added new VIB, access via svs vibmgr." << new_vib_id << std::endl;
}

void visual_input_buffer_manager::cli_add_filepath_visual_input_buffer(const std::vector<std::string>& args, std::ostream& os) {
    if (args.size()<2) {
        os << "Not enough arguments specified." << std::endl;
        return;
    }

    std::string new_vib_id(args[0]);
    std::string new_vib_fp(args[1]);
    add_visual_input_buffer(new_vib_id, new filepath_visual_input_buffer(si, vib_manager_link, new_vib_id, new_vib_fp));

    os << "Added new VIB, access via svs vibmgr." << new_vib_id << std::endl;
}

void visual_input_buffer_manager::cli_del_visual_input_buffer(const std::vector<std::string>& args, std::ostream& os) {
    if (args.empty()) {
        os << "No vib name specified." << std::endl;
        return;
    }

    std::string vib_id(args[0]);
    del_visual_input_buffer(vib_id);

    os << "Deleted VIB " << vib_id << std::endl;
}

void visual_input_buffer_manager::cli_get_visual_input_buffers(const std::vector<std::string>& args, std::ostream& os) {
    os << "Visual input buffers:" << std::endl;

    vib_map::iterator vib_itr = visual_input_buffers.begin();
    std::string vib_id;
    visual_input_buffer* vib;

    for (; vib_itr!=visual_input_buffers.end(); vib_itr++) {
        vib_id = vib_itr->first;
        vib = vib_itr->second;
        os << "    " << vib_id;
        if (vib->get_type() == vib_type::filepath) {
            os << " <- " << ((filepath_visual_input_buffer*)vib)->get_filepath();
        }
    }
}
