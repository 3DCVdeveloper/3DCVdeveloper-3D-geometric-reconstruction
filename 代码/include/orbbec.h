#pragma once
#include <OpenNI.h>

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Orbbec
{
    openni::Device mDevice;
    openni::VideoStream mDepthStream; // depth
    cv::VideoCapture mCapture;        // color

public:
    Orbbec();
    void getFrames(cv::Mat &rgb, cv::Mat &depth);
};