#ifndef VISUAL_MATCHING_H
#define VISUAL_MATCHING_H
#ifdef ENABLE_OPENCV

#include <opencv2/opencv.hpp>
#include "image.h"

namespace visual_matching {
    namespace opencv {
        double ssim_compare(opencv_image* a, opencv_image* b);
        double psnr_compare(opencv_image* a, opencv_image* b);
        double simple_template_compare(opencv_image* a, opencv_image* b);
    }
}

#endif
#endif