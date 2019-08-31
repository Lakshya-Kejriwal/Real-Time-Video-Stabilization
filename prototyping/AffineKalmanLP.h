#ifndef AFFINE_KALMAN_LP_H
#define AFFINE_KALMAN_LP_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

class AKLP_VideoStab
{
public:
    AKLP_VideoStab();
    cv::VideoCapture capture;
    
    // Create some random colors
    std::vector<cv::Scalar> colors;

    cv::Mat frame2;
    cv::Mat frame1;

    int k;

    const int HORIZONTAL_BORDER_CROP = 30;

    cv::Mat smoothedMat;
    cv::Mat affine;

    cv::Mat smoothedFrame;

    double dx ;
    double dy ;
    double da ;
    double ds_x ;
    double ds_y ;

    double sx ;
    double sy ;

    double scaleX ;
    double scaleY ;
    double thetha ;
    double transX ;
    double transY ;

    double diff_scaleX ;
    double diff_scaleY ;
    double diff_transX ;
    double diff_transY ;
    double diff_thetha ;

    double errscaleX ;
    double errscaleY ;
    double errthetha ;
    double errtransX ;
    double errtransY ;

    double Q_scaleX ;
    double Q_scaleY ;
    double Q_thetha ;
    double Q_transX ;
    double Q_transY ;

    double R_scaleX ;
    double R_scaleY ;
    double R_thetha ;
    double R_transX ;
    double R_transY ;

    double sum_scaleX ;
    double sum_scaleY ;
    double sum_thetha ;
    double sum_transX ;
    double sum_transY ;
    
    float p_diff_x;
    float p_diff_y;

    cv::Mat stabilize(cv::Mat frame_1 , cv::Mat frame_2);
    void Kalman_Filter(double *scaleX , double *scaleY , double *thetha , double *transX , double *transY);
};

#endif // AFFINE_KALMAN_LP_H
