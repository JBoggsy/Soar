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
// OpenCV
/////////
#include <opencv2/opencv.hpp>


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


class visual_input_buffer {
protected:
    const static int MAX_SIZE = 8;

    int size;
    int newest_update;
    int oldest_update;
    visual_buffer_frame** buffer;

    Symbol* visual_buffer_link;
    Symbol* size_sym;
    wme*    size_wme;
    Symbol* newest_update_sym;
    wme*    newest_update_wme;
    Symbol* oldest_update_sym;
    wme*    oldest_update_wme;
    Symbol* frames_link;

    soar_interface* si;

public:
    visual_input_buffer(soar_interface* si, Symbol* vsm_link);
    ~visual_input_buffer();

    bool add_new_frame(const cv::Mat& new_image);
    opencv_image* get_frame();
    opencv_image* get_frame(int index);

    int get_size() { return size; }
    int get_max_size() { return MAX_SIZE; }

    int get_newest_update_time() { return newest_update; }
    int get_oldest_update_time() { return oldest_update; }
};


class filepath_visual_input_buffer : public visual_input_buffer {
    private:
        std::string filepath;

        bool file_exists(std::string fp);

    public:
        filepath_visual_input_buffer(soar_interface* si, Symbol* vsm_link, std::string fp);
        ~filepath_visual_input_buffer();

        void proxy_get_children(std::map<std::string, cliproxy*>& c);
        void proxy_use_sub(const std::vector<std::string>& args, std::ostream& os);

        void cli_setfile(const std::vector<std::string>& args, std::ostream& os);
        void cli_load(const std::vector<std::string>& args, std::ostream& os);
        void cli_inject(const std::vector<std::string>& args, std::ostream& os);


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


class visual_input_buffer_manager {
    private:
        std::map<int, visual_input_buffer*> visual_input_buffers;

    public:
        visual_input_buffer_manager();
        ~visual_input_buffer_manager();
        visual_input_buffer* get_vib(int id) { return visual_input_buffers[id]; }
};

#endif
