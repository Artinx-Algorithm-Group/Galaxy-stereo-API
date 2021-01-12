#include <string>
#include <sstream>
#include <iostream>
#include <thread>
#include <csignal>
#include <chrono>

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include "StereoCamera/StereoCameraMutiThread.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::stringstream;
using std::thread;
using std::signal;

using std::chrono::steady_clock;
using std::chrono::microseconds;
using std::chrono::duration_cast;

using boost::filesystem::path;
using boost::filesystem::create_directories;
using boost::filesystem::absolute;

using cv::Mat;
using cv::Vec3b;
using cv::randu;
using cv::Scalar;
using cv::imwrite;
using cv::waitKey;
using cv::cvtColor;

using StereoCamera::StereoStatus;
using StereoCamera::StereoMultithread;
using StereoCamera::StereoFrame;

namespace {
    const double kExposureTime = 20.0;
    const double kFrameRate = 150.0;

    // c-style string serial number for code compatibility
    char left_cam_serial_num[] = "KE0200080465";
    char right_cam_serial_num[] = "KE0200080462";

    bool stop_flag = false;
}

void SigintHandler(int sig) {
    stop_flag = true;
    cout << "SIGINT received, exiting" << endl;
}

int main(int argc, char const *argv[]){
    if(argc != 3){
        cerr << "[usage] ./MutiThreadRecordDataRect <path to calibration data> <path to target directory>" << endl;
        return EXIT_FAILURE;
    }

    stringstream left_dst_dir, right_dst_dir;
    left_dst_dir << argv[2] << "/left";
    right_dst_dir << argv[2] << "/right";

    path left_dst_path_abs = absolute(path(left_dst_dir.str()));
    path right_dst_path_abs = absolute(path(right_dst_dir.str()));

    create_directories(left_dst_path_abs);
    create_directories(right_dst_path_abs);

    cout << "Left path: " << left_dst_path_abs.string() << endl;
    cout << "Right path: " << right_dst_path_abs.string() << endl;
    cout << "Stereo calibration data path: " << argv[1] << endl;

    // Mat img(100,100,CV_8UC3);
    // randu(img, Scalar(0, 0, 0), Scalar(255, 255, 255));

    // stringstream img_path;
    // img_path << left_dst_path_abs.string() << "/test.png";
    // imwrite(img_path.str(), img);

    StereoCamera::StereoMultithread stereo;

    stereo.StereoInitSerialNumber(left_cam_serial_num, right_cam_serial_num);

    stereo.SetStereoCamExposureTime(kExposureTime);
    stereo.SetStereoCamFrameRate(kFrameRate);

    StereoStatus status = stereo.LoadStereoCaliData(argv[1]);
    if(status != StereoStatus::kStereoSuccess) {
        cerr << "Load stereo calibration data failed" << endl;
        return EXIT_FAILURE;
    }

    stereo.ComputeRectParam();

    stereo.StartStereoStream();

    signal(SIGINT, SigintHandler);
    thread img_acquire_thread(&StereoMultithread::MutithreadCaptureTask, &stereo);

    while (!stop_flag){
        if(stereo.IsBufferEmpty()){
            continue;
        }
        StereoFrame stereo_frame;
        Mat combined_img;
        stereo.AcquireStereoFrameFromThread(stereo_frame);

        steady_clock::time_point cap_start = steady_clock::now();

        cvtColor(stereo_frame.left_img(), stereo_frame.left_img(), cv::COLOR_RGB2BGR);
        cvtColor(stereo_frame.right_img(), stereo_frame.right_img(), cv::COLOR_RGB2BGR);
        hconcat(stereo_frame.left_img(), stereo_frame.right_img(), combined_img);

        imshow("out", combined_img);

        stringstream left_img_path, right_img_path;
        left_img_path << left_dst_path_abs.string() << "/" << stereo_frame.timestamp() << ".png";
        right_img_path << right_dst_path_abs.string() << "/" << stereo_frame.timestamp() << ".png";

        imwrite(left_img_path.str(), stereo_frame.left_img());
        imwrite(right_img_path.str(), stereo_frame.right_img());

        // TODO:
        // Bug when the data pending time is high
        // Maybe check cpp queue implementation method

        waitKey(1);

        steady_clock::time_point right_cam_cap_end = steady_clock::now();
        microseconds cap_time = duration_cast<microseconds>(right_cam_cap_end - cap_start);
        cout << "Image write time: " << cap_time.count() << " us" << endl;
    }

    stereo.TerminateTask();

    img_acquire_thread.join();

    stereo.StereoStreamOff();
    stereo.StereoClose();

    return EXIT_SUCCESS;
}
