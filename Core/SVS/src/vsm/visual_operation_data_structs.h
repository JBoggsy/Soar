#ifndef DATA_DICT_H
#define DATA_DICT_H

#include <unordered_map>
#include <vector>
#include <string>
class opencv_image;

/**
 * The `data_dict` type provides string-to-anything mapping for giving/getting
 * data from visual operations.
 */
typedef std::unordered_map<std::string, void*>      data_dict;
typedef std::unordered_map<int, opencv_image*>      int_image_map;

#endif