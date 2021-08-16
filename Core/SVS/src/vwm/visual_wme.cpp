////////////////////
// BEGIN PREAMBLE //
////////////////////
#include "visual_wme.h"

//////////////////
// END PREAMBLE //
//////////////////

////////////////
// IMAGE_VWME //
////////////////

// Set the image by cropping to the non-transparent content
void image_vwme::set_image(cv::Mat* new_image) {
    cv::Mat binarized, final_img;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Rect bbox;
    int top, right, bottom, left;

    // convert to 1 channel grayscale, then threshold and find contour bbox
    cv::cvtColor(*new_image, binarized, cv::COLOR_BGR2GRAY);
    cv::threshold(binarized, binarized, 1, 255, cv::THRESH_BINARY);
    cv::findContours(binarized, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    bbox = cv::boundingRect(contours[0]);
    
    // create a ROI and copy image data from that
    cv::Mat cropped(*new_image, bbox);
    cropped.copyTo(final_img);

    image.set_image(&final_img);
}