#include <iostream>

#include <opencv2/opencv.hpp>

#include "image.h"

////////////////
// IMAGE_BASE //
////////////////

// Sets the source string and notifies any listening image_descriptors
// of the change so that it will be reflected in the image wme
void image_base::set_source(std::string src) {
    source = src;
    notify_listeners();
}

// Makes the given image descriptor respond to updates in this image
void image_base::add_listener(image_descriptor* id) {
    listeners.push_back(id);
}

// Stops the given image descriptor from responding to updates in this image
void image_base::remove_listener(image_descriptor* id) {
    listeners.remove(id);
}

// Calls the update function in all of the image_descriptors listening
// to this image
void image_base::notify_listeners() {
    for (std::list<image_descriptor*>::iterator i = listeners.begin();
         i != listeners.end(); i++) {
        (*i)->update_desc();
    }
}


/////////////////
// BASIC_IMAGE //
/////////////////

basic_image::basic_image() { source = "none"; }

void basic_image::update_image(std::vector<std::vector<pixel> >& new_img) {
    img_array = new_img;
}

void basic_image::copy_from(basic_image* other) {
    img_array = other->img_array;
    source = "copy";
}

int basic_image::get_width() {
    return img_array.size();
}

int basic_image::get_height() {
    if (img_array.size() == 0) return 0;
    return img_array[0].size();
}

bool basic_image::is_empty() {
    if (img_array.empty()) return true;
    if (img_array[0].empty()) return true;
    return false;
}

bool basic_image::operator==(basic_image& other) {
    if (get_width() != other.get_width()) { return false; }
    if (get_height() != other.get_height()) { return false; }

    pixel this_pixel;
    pixel other_pixel;
    for (int col = 0; col < get_width(); col++) {
        for (int row = 0; row < get_height(); row++) {
            this_pixel = img_array.at(col).at(row);
            other_pixel = other.img_array.at(col).at(row);
            if (this_pixel.r != other_pixel.r ||
                this_pixel.g != other_pixel.g ||
                this_pixel.b != other_pixel.b) { return false; }
        }
    }
    
    return true;
}

float basic_image::compare(basic_image* other) {
    if (get_width() != other->get_width()) { return 0.0f; }
    if (get_height() != other->get_height()) { return 0.0f; }

    pixel this_pixel;
    pixel other_pixel;
    int diff_r, diff_g, diff_b;
    int diff_total = 0;
    for (int col = 0; col < get_width(); col++) {
        for (int row = 0; row < get_height(); row++) {
            this_pixel = img_array.at(col).at(row);
            other_pixel = other->img_array.at(col).at(row);
            diff_r = abs(this_pixel.r - other_pixel.r);
            diff_g = abs(this_pixel.g - other_pixel.g);
            diff_b = abs(this_pixel.b - other_pixel.b);
            diff_total += diff_r + diff_g + diff_b;
        }
    }

    int max_error = get_width() * get_height() * 3 * 255;
    float error_quotient = (float)diff_total/(float)max_error;
    float error = 1.0f - error_quotient;
    return error;
}


//////////////////
// OPENCV_IMAGE //
//////////////////
#ifdef ENABLE_OPENCV
opencv_image::opencv_image() { source = "none"; }

void opencv_image::update_image(const cv::Mat& new_img) {
    bool prev_empty = is_empty();
    _img = new cv::Mat;
    *_img = new_img.clone();
    if (is_empty() != prev_empty) notify_listeners();
}

void opencv_image::copy_from(opencv_image* other) {
    cv::Mat* other_image = other->get_image();
    update_image(*other_image);
    source = "copy";
}

bool opencv_image::is_empty() {
    if (_img == NULL) { return true; }
    return _img->empty();
}

bool opencv_image::operator==(opencv_image& other) {
    cv::Mat difference_mat = *_img - *other._img;
    cv::Scalar channel_diffs = cv::sum(difference_mat);
    double total_diff = channel_diffs.val[0] + channel_diffs.val[1] + channel_diffs.val[2] + channel_diffs.val[3];
    return (total_diff == 0.0);
}

float opencv_image::compare(opencv_image* other) {
    /// SOURCE: https://docs.opencv.org/2.4/doc/tutorials/highgui/video-input-psnr-ssim/video-input-psnr-ssim.html#videoinputpsnrmssim
    const double C1 = 6.5025, C2 = 58.5225;
    /***************************** INITS **********************************/
    int d = CV_32F;

    cv::Mat I1, I2;
    _img->convertTo(I1, d);            // cannot calculate on one byte large values
    other->_img->convertTo(I2, d);

    cv::Mat I2_2   = I2.mul(I2);        // I2^2
    cv::Mat I1_2   = I1.mul(I1);        // I1^2
    cv::Mat I1_I2  = I1.mul(I2);        // I1 * I2

    /*************************** END INITS **********************************/

    cv::Mat mu1, mu2;                   // PRELIMINARY COMPUTING
    cv::GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
    cv::GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);

    cv::Mat mu1_2   =   mu1.mul(mu1);
    cv::Mat mu2_2   =   mu2.mul(mu2);
    cv::Mat mu1_mu2 =   mu1.mul(mu2);

    cv::Mat sigma1_2, sigma2_2, sigma12;

    cv::GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5);
    sigma1_2 -= mu1_2;

    cv::GaussianBlur(I2_2, sigma2_2, cv::Size(11, 11), 1.5);
    sigma2_2 -= mu2_2;

    cv::GaussianBlur(I1_I2, sigma12, cv::Size(11, 11), 1.5);
    sigma12 -= mu1_mu2;

    ///////////////////////////////// FORMULA ////////////////////////////////
    cv::Mat t1, t2, t3;

    t1 = 2 * mu1_mu2 + C1;
    t2 = 2 * sigma12 + C2;
    t3 = t1.mul(t2);                 // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))

    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    t1 = t1.mul(t2);                 // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

    cv::Mat ssim_map;
    cv::divide(t3, t1, ssim_map);        // ssim_map =  t3./t1;

    cv::Scalar mssim = cv::mean(ssim_map);   // mssim = average of ssim map

    return mssim.val[0] + mssim.val[1] + mssim.val[2] + mssim.val[3];
}

#endif


///////////////
// PCL_IMAGE //
///////////////

#ifdef ENABLE_ROS
pcl_image::pcl_image() { source = "none"; }

// Copies a point cloud ROS message into this image container.
void pcl_image::update_image(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& new_img) {
    bool prev_empty = is_empty();

    pc.header = new_img->header;
    pc.width = new_img->width;
    pc.height = new_img->height;
    pc.is_dense = new_img->is_dense;
    pc.sensor_orientation_ = new_img->sensor_orientation_;
    pc.sensor_origin_ = new_img->sensor_origin_;
    pc.points = new_img->points;

    if (is_empty() != prev_empty) notify_listeners();
}

// Copies the point cloud from another image into this image container.
void pcl_image::copy_from(pcl_image* other) {
    pc.header = other->pc.header;
    pc.width = other->pc.width;
    pc.height = other->pc.height;
    pc.is_dense = other->pc.is_dense;
    pc.sensor_orientation_ = other->pc.sensor_orientation_;
    pc.sensor_origin_ = other->pc.sensor_origin_;
    pc.points = other->pc.points;
    source = "copy";
}

bool pcl_image::is_empty() {
    return (pc.width == 0 && pc.height == 0);
}

bool pcl_image::operator==(pcl_image& other) {
    if (!pc.isOrganized() || ~other.pc.isOrganized()) { return false; }
    if (pc.width != other.pc.width) { return false; }
    if (pc.height != other.pc.height) { return false; }

    for (int row = 0; row < pc.width; row++) {
        for (int col = 0; col < pc.height; col++) {
            if (pcl::squaredEuclideanDistance(pc(col, row), other.pc(col, row)) > 0.0f) {
                return false;
            }
        }
    }

    return true;
}

#endif


//////////////////////
// IMAGE_DESCRIPTOR //
//////////////////////

// These are the names of the attributes of an image descriptor in WM
const std::string image_descriptor::source_tag = "source";
const std::string image_descriptor::empty_tag = "empty";

// Constructor requires a pointer to the soar_interface to be able to
// add/delete WMEs, the image link symbol that will serve as the root for
// the image info, and an image to listen to
image_descriptor::image_descriptor(soar_interface* si, Symbol* ln, image_base* im)
    : img(im), link(ln), si(si), source("none"), empty("true")
{
    img->add_listener(this);

    // Creates the image ^source none wme to start
    source_wme = si->make_wme(link,
                              source_tag,
                              source);
    // Creates the image ^empty true wme to start
    empty_wme = si->make_wme(link,
                             empty_tag,
                             empty);
    update_desc();
}

// Updates the wmes if something has changed in image that this
// is listening to
void image_descriptor::update_desc() {
    // First check if the image either became non-empty after being
    // empty or vice versa and update wme if so
    bool updated = false;
    if (empty == "true" && !img->is_empty()) {
        empty = "false";
        updated = true;
    } else if (empty == "false" && img->is_empty()) {
        empty = "true";
        updated = true;
    }
    if (updated) {
        // XXX: I only see add/delete functionality in soar_interface.
        //      Is this the right way to change the value?
        si->remove_wme(empty_wme);
        empty_wme = si->make_wme(link,
                                 empty_tag,
                                 empty);
    }

    // Then check if the image's source string has changed and update
    // wme if so
    if (source != img->get_source()) {
        // XXX: I only see add/delete functionality in soar_interface.
        //      Is this the right way to change the value?
        source = img->get_source();
        si->remove_wme(source_wme);
        source_wme = si->make_wme(link,
                                  source_tag,
                                  source);
    }
}
