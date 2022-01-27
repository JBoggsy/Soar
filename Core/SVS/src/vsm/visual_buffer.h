#ifndef VISUAL_BUFFER_H
#define VISUAL_BUFFER_H

//////////////
// PREAMBLE //
//////////////
// standard lib includes
////////////////////////
#include <vector>
#include <time.h>
// Soar includes
///////////////
#include "soar_interface.h"
#include "image.h"


class visual_buffer_frame
{
private:
    soar_interface* si_;
    opencv_image* image_;
    std::time_t timestamp_time_t_;
    int timestamp_;
    int index_;

    Symbol* frame_link_;
    Symbol* timestamp_sym_;
    wme* timestamp_wme_;
    Symbol* index_sym_;
    wme* index_wme_;
public:
    visual_buffer_frame(soar_interface* si, const cv::Mat& image, Symbol* frame_link);
    ~visual_buffer_frame();
    opencv_image* get_image() { return image_; }
    int get_timestamp() { return timestamp_; }
    void increment_index();
};


class visual_buffer {
private:
    const static int MAX_SIZE_ = 8;

    int size_;
    int newest_update_;
    int oldest_update_;
    visual_buffer_frame** buffer_;

    Symbol* visual_buffer_link_;
    Symbol* size_sym_;
    wme*    size_wme_;
    Symbol* newest_update_sym_;
    wme*    newest_update_wme_;
    Symbol* oldest_update_sym_;
    wme*    oldest_update_wme_;
    Symbol* frames_link_;

    soar_interface* si_;

public:
    visual_buffer(soar_interface* si, Symbol* vsm_link);
    ~visual_buffer();

    bool add_new_frame(const cv::Mat& new_image);
    opencv_image* get_frame();
    opencv_image* get_frame(int index);

    int get_size() { return size_; }
    int get_max_size() { return MAX_SIZE_; }
    
    int get_newest_update_time() { return newest_update_; }
    int get_oldest_update_time() { return oldest_update_; }
};

#endif