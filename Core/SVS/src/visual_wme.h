#ifndef VISUAL_WME_H
#define VISUAL_WME_H

#include <vector>
#include "image.h"

class visual_wme
{
private:
protected:
    std::string id;
    visual_wme* parent;
    
public:
    visual_wme() {}
    visual_wme(std::string id, visual_wme* parent) {
        this->id = id;
        this->parent = parent;
    }
    std::string get_id() { return id; }
};


class composition_vwme: public visual_wme
{
private:
    std::vector<visual_wme*> children;
public:
    composition_vwme() {}
    composition_vwme(std::string id, visual_wme* parent) {
        this->id = id;
        this->parent = parent;
    }

    void add_child(visual_wme* child);
    void del_child(std::string id);

    void compose();
};


class primitive_vwme: public visual_wme
{
private:
public:
    primitive_vwme() {}
    primitive_vwme(std::string _id, visual_wme* _parent);
};


class image_vwme: public primitive_vwme
{
private:
    opencv_image image;
public:
    image_vwme() {}
    image_vwme(std::string id, visual_wme* parent) {
        this->id = id;
        this->parent = parent;
    }

    cv::Mat* get_image() { return image.get_image(); }
    void set_image(cv::Mat* new_image) { image.set_image(new_image); }

};
#endif