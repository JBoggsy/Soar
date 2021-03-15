#include <unordered_map>
#include <cmath>

#include "imagination.h"

#ifdef ENABLE_OPENCV
#include <opencv2/opencv.hpp>


imagination_opencv::imagination_opencv() {}

///////////////////
// MANIPULATION //
/////////////////
void imagination_opencv::get_image(opencv_image& output) {
    _draw_canvas();
    output.update_image(canvas);
}

int imagination_opencv::add_percept(opencv_image new_percept) {
    dirty = true;
    return add_percept(new_percept, 0, 0);
}
int imagination_opencv::add_percept(opencv_image new_percept, int x, int y) {
    int new_percept_ID = next_percept_id;
    
    imagination_opencv_percept_metadata new_percept_metadata;
    new_percept_metadata.x = x;
    new_percept_metadata.y = y;
    new_percept_metadata.rotation = 0;
    new_percept_metadata.h_mirror = false;
    new_percept_metadata.v_mirror = false;

    percepts[new_percept_ID] = new_percept;
    metadata[new_percept_ID] = new_percept_metadata;

    next_percept_id++;
    dirty = true;
    return new_percept_ID;
}

void imagination_opencv::remove_percept(int percept_ID) {
    percepts.erase(percept_ID);
    metadata.erase(percept_ID);
    dirty = true;
}

void imagination_opencv::translate_percept(int percept_ID, int dX, int dY) {
    int old_x = metadata[percept_ID].x;
    int old_y = metadata[percept_ID].y;

    int new_x = old_x + dX;
    int new_y = old_y + dY;

    move_percept(percept_ID, new_x, new_y);
    dirty = true;
}

void imagination_opencv::move_percept(int percept_ID, int new_x, int new_y) {
    metadata[percept_ID].x = new_x;
    metadata[percept_ID].y = new_y;

    dirty = true;
}

void imagination_opencv::rotate_percept_rad(int percept_ID, double rads) {
    float degs = rads * 180.0/M_PI;
    rotate_percept_deg(percept_ID, degs);
    dirty = true;
}
void imagination_opencv::rotate_percept_deg(int percept_ID, double degs) {
    metadata[percept_ID].rotation = degs;
    dirty = true;
}

void imagination_opencv::flip_percept_horiz(int percept_ID) {
    metadata[percept_ID].h_mirror = !metadata[percept_ID].h_mirror;
    dirty = true;
}
void imagination_opencv::flip_percept_vert(int percept_ID) {
    metadata[percept_ID].v_mirror = !metadata[percept_ID].v_mirror;
    dirty = true;
}


//////////////
// DRAWING //
////////////

void imagination_opencv::_generate_canvas() {
    typedef std::unordered_map<int, opencv_image>::iterator percept_iterator;
    
    int left_bound = 0;
    int right_bound = 0;
    int top_bound = 0;
    int bottom_bound = 0;
    percept_iterator percept = percepts.begin();
    for (; percept != percepts.end(); percept++) {

        int pcept_width = percept->second.get_image()->cols;
        int pcept_height = percept->second.get_image()->rows;

        int pcept_x = metadata[percept->first].x;
        int pcept_y = metadata[percept->first].y;

        int pcept_left_bound = pcept_x - int(ceil(pcept_width/2.0));
        int pcept_right_bound = pcept_x + int(ceil(pcept_width/2.0));
        int pcept_top_bound = pcept_y - int(ceil(pcept_height/2.0));
        int pcept_bottom_bound = pcept_y + int(ceil(pcept_height/2.0));

        left_bound = MIN(left_bound, pcept_left_bound);
        right_bound = MAX(right_bound, pcept_right_bound);
        top_bound = MIN(top_bound, pcept_top_bound);
        bottom_bound = MAX(bottom_bound, pcept_bottom_bound);
    }

    int canvas_width = right_bound - left_bound;
    int canvas_height = bottom_bound - top_bound;
    canvas_width = MAX(canvas_width, 2);
    canvas_height = MAX(canvas_height, 2);

    canvas = cv::Mat::zeros(canvas_height, canvas_width, CV_8UC3);
    origin = cv::Point2i(-left_bound, -top_bound);
}

void imagination_opencv::_draw_percept(opencv_image percept, imagination_opencv_percept_metadata mdata) {
    cv::Mat drawn_img = percept.get_image()->clone();
    cv::Point2f percept_center(drawn_img.cols/2.0, drawn_img.rows/2.0);

    cv::Mat rotation_mat = cv::getRotationMatrix2D(percept_center, -1*mdata.rotation, 1.0);
    cv::warpAffine(drawn_img, drawn_img, rotation_mat, cv::Size(drawn_img.cols, drawn_img.rows));

    if (mdata.h_mirror) {
        cv::flip(drawn_img, drawn_img, 0);
    }
    if (mdata.v_mirror) {
        cv::flip(drawn_img, drawn_img, 1);
    }

    int left_bound = mdata.x - drawn_img.cols/2;
    int top_bound = mdata.y - drawn_img.rows/2;

    drawn_img.copyTo(canvas(cv::Rect(left_bound+origin.x, top_bound+origin.y, drawn_img.cols, drawn_img.rows)));

}

void imagination_opencv::_draw_canvas() {
    _generate_canvas();
    typedef std::unordered_map<int, opencv_image>::iterator percept_iterator;
    
    percept_iterator percept = percepts.begin();
    for (; percept != percepts.end(); percept++) {
        _draw_percept(percept->second, metadata[percept->first]);
    }

    dirty = false;
}

#endif