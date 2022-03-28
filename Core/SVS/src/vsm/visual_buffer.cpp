#include "visual_buffer.h"

/////////////////////////
// VISUAL BUFFER FRAME //
/////////////////////////

visual_buffer_frame::visual_buffer_frame(soar_interface* si, const cv::Mat& image, wme* frame_link) {
    index_ = 0;
    si_ = si;
    image_ = new opencv_image();
    image_->update_image(image);
    timestamp_time_t_ = std::time(NULL);
    timestamp_ = (int)timestamp_time_t_;
    frame_wme_ = frame_link;
    frame_sym_ = si_->get_wme_val(frame_wme_);

    timestamp_sym_ = si->make_sym(timestamp_);
    timestamp_wme_ = si_->make_wme(frame_sym_, std::string("timestamp"), timestamp_sym_);

    index_sym_ = si_->make_sym(index_);
    index_wme_ = si_->make_wme(frame_sym_, std::string("index"), index_sym_);
}

visual_buffer_frame::~visual_buffer_frame()
{
    delete image_;
    si_->remove_wme(timestamp_wme_);
    si_->remove_wme(index_wme_);
    si_->remove_wme(frame_wme_);
}

void visual_buffer_frame::increment_index() {
    index_++;
    si_->remove_wme(index_wme_);
    index_sym_ = si_->make_sym(index_);
    index_wme_ = si_->make_wme(frame_sym_, std::string("index"), index_sym_);
}


///////////////////
// VISUAL BUFFER //
///////////////////

visual_buffer::visual_buffer(soar_interface* si, Symbol* vsm_link)
    : si_(si) {
        buffer_ = new visual_buffer_frame*[MAX_SIZE_];
        for (int i=0; i<MAX_SIZE_; i++) {
            buffer_[i] = NULL;
        }
        size_ = 0;

        visual_buffer_link_ = si->get_wme_val(si->make_id_wme(vsm_link, "visual-buffer"));

        size_sym_ = si_->make_sym(size_); 
        size_wme_ = si_->make_wme(visual_buffer_link_, std::string("size"), size_sym_);

        newest_update_sym_ = si_->make_sym(newest_update_); 
        newest_update_wme_ = si_->make_wme(visual_buffer_link_, std::string("newest-update"), newest_update_sym_);

        oldest_update_sym_ = si_->make_sym(oldest_update_); 
        oldest_update_wme_ =  si_->make_wme(visual_buffer_link_, std::string("oldest-update"), oldest_update_sym_);

        frames_link_ = si_->get_wme_val(si_->make_id_wme(visual_buffer_link_, std::string("frames")));
}

visual_buffer::~visual_buffer() {
    for (int i=0; i<size_; i++) {
        delete buffer_[i];
    }
    delete buffer_;

    si_->remove_wme(size_wme_);
    si_->remove_wme(newest_update_wme_);
    si_->remove_wme(oldest_update_wme_);
    si_->del_sym(frames_link_);
}

bool visual_buffer::add_new_frame(const cv::Mat& new_image) {
    if (size_ != MAX_SIZE_) {
        size_ ++;
        si_->remove_wme(size_wme_);
        size_sym_ = si_->make_sym(size_); 
        size_wme_ = si_->make_wme(visual_buffer_link_, std::string("size"), size_sym_);
    }

    // Eject the last element in the percept buffer
    if (buffer_[MAX_SIZE_-1] != NULL) {
        delete buffer_[MAX_SIZE_-1];
        buffer_[MAX_SIZE_-1] = NULL;

    }

    // Move the remaining elements up
    for (int i=MAX_SIZE_-2; i>=0; i--) {
        if (buffer_[i] == NULL) { continue; }
        buffer_[i+1] = buffer_[i];
        buffer_[i+1]->increment_index();
    } 
    
    // Create the latest element
    wme* frame_link = si_->make_id_wme(frames_link_, std::string("frame"));
    buffer_[0] = new visual_buffer_frame(si_, new_image, frame_link);

    // Update the "updated" values
    newest_update_ = buffer_[0]->get_timestamp();
    si_->remove_wme(newest_update_wme_);
    newest_update_sym_ = si_->make_sym(newest_update_); 
    newest_update_wme_ = si_->make_wme(visual_buffer_link_, std::string("newest-update"), newest_update_sym_);

    oldest_update_ = buffer_[size_-1]->get_timestamp();
    si_->remove_wme(oldest_update_wme_);
    oldest_update_sym_ = si_->make_sym(oldest_update_);
    oldest_update_wme_ = si_->make_wme(visual_buffer_link_, std::string("oldest-update"), oldest_update_sym_);
    return true;
}

opencv_image* visual_buffer::get_frame() {
    return buffer_[0]->get_image();
}

opencv_image* visual_buffer::get_frame(int index) {
    if (index<0 || index >= size_) {
        return NULL;
    }
    return buffer_[index]->get_image();
}