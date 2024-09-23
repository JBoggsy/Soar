#pragma once
// standard includes
#include <string>
// third-party includes
#include <opencv2/opencv.hpp>

// forward declarations
class torch_module_wrapper;

class neural_network 
{
private:
    torch_module_wrapper* module;
public:
    neural_network();
    ~neural_network();
    void load_traced_script(std::string traced_script_path);
    cv::Mat forward(cv::Mat input);
};
