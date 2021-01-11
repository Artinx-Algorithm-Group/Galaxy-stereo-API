project(stereo_driver)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(OpenCV_DIR /home/artinx001/3rdparty/opencv-3.4.12/build)
find_package(OpenCV 3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/inc
)

add_executable(
    StereoImgCaptureSampleRect
        ${PROJECT_SOURCE_DIR}/src/GxCamera.cc
        ${PROJECT_SOURCE_DIR}/src/StereoCamera.cc
        ${PROJECT_SOURCE_DIR}/sample/StereoImgCaptureRect.cc
)

target_link_libraries(
    StereoImgCaptureSampleRect 
        ${OpenCV_LIBS}
        libgxiapi.so          # /usr/lib/libgxiapi.so
)

add_executable(
    StereoTestFPS
        ${PROJECT_SOURCE_DIR}/src/GxCamera.cc
        ${PROJECT_SOURCE_DIR}/src/StereoCamera.cc
        ${PROJECT_SOURCE_DIR}/sample/StereoTestFPS.cc
)

target_link_libraries(
    StereoTestFPS 
        ${OpenCV_LIBS}
        libgxiapi.so          # /usr/lib/libgxiapi.so
)

add_executable(
    MutiThreadCapture
        ${PROJECT_SOURCE_DIR}/src/GxCamera.cc
        ${PROJECT_SOURCE_DIR}/src/StereoCamera.cc
        ${PROJECT_SOURCE_DIR}/src/StereoCameraMutiThread.cc
        ${PROJECT_SOURCE_DIR}/sample/MutiThreadCapture.cc
)

target_link_libraries(
    MutiThreadCapture 
        ${OpenCV_LIBS}
        libgxiapi.so          # /usr/lib/libgxiapi.so
        Threads::Threads
)

add_executable(
    MutiThreadRecordDataRect
        ${PROJECT_SOURCE_DIR}/src/GxCamera.cc
        ${PROJECT_SOURCE_DIR}/src/StereoCamera.cc
        ${PROJECT_SOURCE_DIR}/src/StereoCameraMutiThread.cc
        ${PROJECT_SOURCE_DIR}/sample/MutiThreadRecordDataRect.cc
)

target_link_libraries(
    MutiThreadRecordDataRect 
        ${OpenCV_LIBS}
        libgxiapi.so          # /usr/lib/libgxiapi.so
        Threads::Threads
        Boost::filesystem
)
