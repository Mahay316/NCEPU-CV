cmake_minimum_required(VERSION 3.15)
project(robomaster)

set(CMAKE_CXX_STANDARD 11)

# settings for opencv
set(OpenCV_DIR "D:\\opencv\\mingw-build\\install")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(robomaster cam_calib.cpp)

# add libs the project needs
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui opencv_calib3d)
target_link_libraries(robomaster ${OpenCV_LIBS})