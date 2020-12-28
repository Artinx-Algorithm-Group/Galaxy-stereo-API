/**
 * @file    StereoCamera.hpp
 * @brief   Class decleration for stereo driver
 * @author  Jing Yonglin
 * @mail:   11712605@mail.sustech.edu.cn
 *          yonglinjing7@gmail.com
*/

#ifndef _STEREO_CAMERA_HPP_
#define _STEREO_CAMERA_HPP_

#include <chrono>

#include <opencv2/opencv.hpp>

#include "GxCamera/GxCamera.hpp"

namespace StereoCamera{

enum class StereoStatus{
    kStereoSuccess = 0,
    kCameraDriverError,
    kSetStereoExposureFail,
    kSetStereoFrameRateFail,
    kStreamStartFail,
    kExposureTimeOutOfBound,
    kFrameRateOutOfBound,
    kGetLeftColorImgFail,
    kGetRightColorImgFail
};

class Stereo{

public:
    ~Stereo() {};

    StereoStatus StereoInitSerialNumber(char *left_cam_serial_num, 
                                        char *right_cam_serial_num);

    StereoStatus SetLeftCamExposureTime(const uint32_t exposure_time);
    StereoStatus SetRightCamExposureTime(const uint32_t exposure_time);
    StereoStatus SetStereoCamExposureTime(const uint32_t exposure_time);

    StereoStatus SetLeftCamFrameRate(const uint16_t frame_rate);
    StereoStatus SetRightCamFrameRate(const uint16_t frame_rate);
    StereoStatus SetStereoCamFrameRate(const uint16_t frame_rate);

    StereoStatus StartLeftCamStream();
    StereoStatus StartRightCamStream();
    StereoStatus StartStereoStream();

    StereoStatus GetColorImgStereo(cv::Mat &left_img, cv::Mat &right_img, double &timestamp);

    StereoStatus StereoStreamOff();
    StereoStatus StereoClose();

private:
    GxCamera left_cam_;
    GxCamera right_cam_;

    std::chrono::steady_clock::time_point capture_start_time_ = std::chrono::steady_clock::now();

    bool is_timestamp_init_ = false;
};

}

#endif