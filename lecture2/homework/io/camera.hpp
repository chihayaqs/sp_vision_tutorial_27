#pragma once

#include "hikrobot/include/MvCameraControl.h"
#include <opencv2/opencv.hpp>

class Camera
{
public:
    Camera();
    ~Camera();

    bool read(cv::Mat & img);

private:
    void * handle_ = nullptr;
    cv::Mat transfer(MV_FRAME_OUT & raw);
};