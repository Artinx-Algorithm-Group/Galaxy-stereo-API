# Stereo Rectification with OpenCV

Explanation on the rectification method in this driver

## Camera intrinsics , extrinsics and distortion parameters calibration

* intrinsics: 3x3 intrinsics matrix for each camera
* extrinsics: 3x3 rotation matrix and 3x1 translation vector describing pose transformation form the first camera to the second camera
* distortion: 1x5 vector describing the distortion pattern of camera

A quick method is to use [Stereo Camera Calibrator App](https://www.mathworks.com/help/vision/ug/stereo-camera-calibrator-app.html). **Note that in the APP the output intrinsics matrix and rotation matrix need to be transposed to be used with OpenCV**

Another method is to use the function [stereoCalibrate](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga91018d80e2a93ade37539f01e6f07de5).

After obtaining intrinsics, extrinsics and distortion parameters for each camera, put them in a yaml format file for the driver to read. A sample is provided in `sample_cali_info` [folder](../sample_cali_info).

## Compute rectification parameters

Use OpenCV function [stereoRectify](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga617b1685d4059c6040827800e72ad2b6)

Note that `P1` and `P2` are the new instrinsics matrix for camera 1 and camera 2. Use `P.rowRange(0,3).colRange(0,3)` to get the 3x3 intrinsics matrix. For more info, please see the documentation.

With the output from `stereoRectify`, we can use [initUndistortRectifyMap](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a) to compute a map from original image to rectified image.

## Apply  rectification

Use [remap](https://docs.opencv.org/master/da/d54/group__imgproc__transform.html#gab75ef31ce5cdfb5c44b6da5f3b908ea4) with the previous `initUndistortRectifyMap` output to map the original image to the rectification image.