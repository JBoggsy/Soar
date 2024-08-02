#ifndef VISUAL_INPUT_BUFFER_H
#define VISUAL_INPUT_BUFFER_H

//////////////
// PREAMBLE //
//////////////
// standard lib includes
////////////////////////
#include <vector>
#include <map>
#include <time.h>
#include <string>
// Soar includes
///////////////
#include "soar_interface.h"
#include "image.h"
#include "cliproxy.h"
#include "forward.h"
// OpenCV
/////////
#include <opencv2/opencv.hpp>



enum vib_type {
    generic,
    filepath
};

class visual_buffer_frame
{
private:
    soar_interface* si;
    opencv_image* image;
    std::time_t timestamp_time_t;
    int timestamp;
    int index;

    Symbol* frame_sym;
    wme* frame_wme;
    Symbol* timestamp_sym;
    wme* timestamp_wme;
    Symbol* index_sym;
    wme* index_wme;
public:
    visual_buffer_frame(soar_interface* si, const cv::Mat& image, wme* frame_link);
    ~visual_buffer_frame();
    opencv_image* get_image() { return image; }
    int get_timestamp() { return timestamp; }
    void increment_index();
};


class visual_input_buffer : public cliproxy {
protected:
    const static int MAX_SIZE = 8;

    soar_interface* si;

    Symbol* visual_buffer_link;
    Symbol* vib_id_sym;
    wme*    vib_id_wme;
    Symbol* size_sym;
    wme*    size_wme;
    Symbol* newest_update_sym;
    wme*    newest_update_wme;
    Symbol* oldest_update_sym;
    wme*    oldest_update_wme;
    Symbol* frames_link;

    std::string vib_id;
    int size;
    int newest_update;
    int oldest_update;
    visual_buffer_frame** buffer;

    vib_type type;


public:
    visual_input_buffer(soar_interface* si, Symbol* vib_manager_link, std::string vib_id);
    virtual ~visual_input_buffer();

    bool add_new_frame(const cv::Mat& new_image);
    opencv_image* get_frame();
    opencv_image* get_frame(int index);

    int get_size() { return size; }
    int get_max_size() { return MAX_SIZE; }
    vib_type get_type() {return type; }

    int get_newest_update_time() { return newest_update; }
    int get_oldest_update_time() { return oldest_update; }

    void proxy_get_children(std::map<std::string, cliproxy*>& c);
    void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);
    void cli_inject(const std::vector<std::string>& args, std::ostream& os);
    void cli_imgdata(const std::vector<std::string>& args, std::ostream& os);
    void cli_save_img(const std::vector<std::string>& args, std::ostream& os);
};


class filepath_visual_input_buffer : public visual_input_buffer {
    private:
        std::string filepath;

        bool file_exists(std::string fp);

    public:
        filepath_visual_input_buffer(soar_interface* si, Symbol* vib_manager_link, std::string vib_id, std::string fp);
        ~filepath_visual_input_buffer();

        std::string get_filepath() { return filepath; }

        void proxy_get_children(std::map<std::string, cliproxy*>& c);
        void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);

        void cli_setfile(const std::vector<std::string>& args, std::ostream& os);
        void cli_load(const std::vector<std::string>& args, std::ostream& os);


        /**
         * @brief Sets the target image file for the load command.
         *
         * @param filepath The target image file. Must exist.
         */
        void setfile(std::string fp) {filepath = fp;}

        /**
         * @brief Loads the target image file into the agent's visual input.
         *
         * @param args The args sent to the command. These are discarded.
         * @param os The output stream to write to.
         */
        void load();
};


class visual_input_buffer_manager : public cliproxy {
    private:
        typedef std::map<std::string, visual_input_buffer*> vib_map;

        soar_interface* si;
        Symbol* svs_link;

        wme* num_vibs_wme;
        vib_map visual_input_buffers;

        Symbol* vib_manager_link;

    public:
        visual_input_buffer_manager(soar_interface* si, Symbol* vib_link);
        ~visual_input_buffer_manager();

        void add_visual_input_buffer(std::string vib_id, visual_input_buffer* new_vib);
        void del_visual_input_buffer(std::string vib_id);

        std::vector<std::string> get_vib_ids();
        visual_input_buffer* get_visual_input_buffer(std::string vib_id) { return visual_input_buffers[vib_id]; }
        int get_num_vibs() { return (int)visual_input_buffers.size(); }

        void proxy_get_children(std::map<std::string, cliproxy*>& c);
        void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);

        void cli_add_visual_input_buffer(const std::vector<std::string>& args, std::ostream& os);
        void cli_add_filepath_visual_input_buffer(const std::vector<std::string>& args, std::ostream& os);
        void cli_del_visual_input_buffer(const std::vector<std::string>& args, std::ostream& os);
        void cli_get_visual_input_buffers(const std::vector<std::string>& args, std::ostream& os);
};

#endif
