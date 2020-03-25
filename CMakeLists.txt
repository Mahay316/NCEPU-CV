cmake_minimum_required(VERSION 3.17)
project(robomaster)

set(CMAKE_CXX_STANDARD 11)

# settings for opencv
set(OpenCV_DIR "D:\\opencv\\mingw-build\\install")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# add subdirectories of the project
# subdirectories also include CMakeLists.txt
add_subdirectory(armor)
add_subdirectory(pose)

add_executable(robomaster main.cpp)

# add libs the project needs
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui)
set(EXTRA_LIBS rm_armor rm_pose)
target_link_libraries(robomaster ${OpenCV_LIBS} ${EXTRA_LIBS})
