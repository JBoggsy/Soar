#ifdef ENABLE_OPENCV

#include "visual_matching.h"


//////////////////////////////////////
// STRUCTURAL SIMILARITY COMPARISON //
//////////////////////////////////////
/// SOURCE: https://docs.opencv.org/2.4/doc/tutorials/highgui/video-input-psnr-ssim/video-input-psnr-ssim.html#videoinputpsnrmssim
double visual_matching::opencv::ssim_compare(opencv_image* a, opencv_image* b) {
    const double C1 = 6.5025, C2 = 58.5225;
    /***************************** INITS **********************************/
    int d = CV_32F;

    cv::Mat I1, I2;
    a->get_image()->convertTo(I1, d);            // cannot calculate on one byte large values
    b->get_image()->convertTo(I2, d);
    
    // Ensure I1 and I2 are the same size by padding with 0s
    int max_height = cv::max(I1.size[0], I2.size[0]);
    int max_width = cv::max(I1.size[1], I2.size[1]);
    cv::copyMakeBorder(I1, I1, (max_height-I1.rows)/2, (max_height-I1.rows)/2, (max_width-I1.cols)/2, (max_width-I1.cols)/2, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));
    cv::copyMakeBorder(I2, I2, (max_height-I2.rows)/2, (max_height-I2.rows)/2, (max_width-I2.cols)/2, (max_width-I2.cols)/2, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));

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


///////////////////////////////////////////
// PEAK SIGNAL-TO-NOISE RATIO COMPARISON //
///////////////////////////////////////////
/// SOURCE: https://docs.opencv.org/2.4/doc/tutorials/highgui/video-input-psnr-ssim/video-input-psnr-ssim.html#videoinputpsnrmssim
double visual_matching::opencv::psnr_compare(opencv_image* a, opencv_image* b) {
    cv::Mat s1;
    cv::Mat I1, I2;
    I1 = *(a->get_image());
    I2 = *(b->get_image());


    // Ensure I1 and I2 are the same size by padding with 0s
    int max_height = cv::max(I1.size[0], I2.size[0]);
    int max_width = cv::max(I1.size[1], I2.size[1]);
    cv::copyMakeBorder(I1, I1, (max_height-I1.rows)/2, (max_height-I1.rows)/2, (max_width-I1.cols)/2, (max_width-I1.cols)/2, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));
    cv::copyMakeBorder(I2, I2, (max_height-I2.rows)/2, (max_height-I2.rows)/2, (max_width-I2.cols)/2, (max_width-I2.cols)/2, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));

    // Blur for generality
    cv::blur(I1, I1, cv::Size(3, 3));
    cv::blur(I2, I2, cv::Size(3, 3));

    cv::absdiff(I1, I2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2

    cv::Scalar s = cv::sum(s1);         // sum elements per channel

    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

    if( sse <= 1e-10) // for small values return zero
        return 150.0;
    else
    {
        double  mse =sse /(double)(I1.channels() * I1.total());
        double psnr = 10.0*log10((255*255)/mse);
        return psnr;
    }
}


//////////////////////////////////////
// SIMPLE TEMPLATE-BASED COMPARISON //
//////////////////////////////////////
// See https://docs.opencv.org/3.4/de/da9/tutorial_template_matching.html
/// NOTE: Matches b onto a
double visual_matching::opencv::simple_template_compare(opencv_image* a, opencv_image* b) {
    cv::Mat base = *(a->get_image());
    cv::Mat tmpl = *(b->get_image());
    cv::Mat base_gray, tmpl_gray;
    cv::Mat result_mat;
    double result;


    // Ensure base and tmpl are the same size by padding with 0s
    int max_height = cv::max(base.size[0], tmpl.size[0]);
    int max_width = cv::max(base.size[1], tmpl.size[1]);
    cv::copyMakeBorder(base, base, (max_height-base.rows)/2, (max_height-base.rows)/2, (max_width-base.cols)/2, (max_width-base.cols)/2, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));
    cv::copyMakeBorder(tmpl, tmpl, (max_height-tmpl.rows)/2, (max_height-tmpl.rows)/2, (max_width-tmpl.cols)/2, (max_width-tmpl.cols)/2, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));

    cv::cvtColor(base, base_gray, cv::COLOR_BGRA2GRAY);
    cv::cvtColor(tmpl, tmpl_gray, cv::COLOR_BGRA2GRAY);
    cv::matchTemplate(base_gray, tmpl_gray, result_mat, cv::TM_CCOEFF_NORMED);
    // printf("Templating result size: %d x %d\n", result_mat.cols, result_mat.rows);
    cv::minMaxLoc(result_mat, NULL, &result, NULL, NULL);

    return result;
}
#endif