# compile source files in this directory into a static library
# named rm_pose

AUX_SOURCE_DIRECTORY(. POSE_SRC)
ADD_LIBRARY(rm_pose ${POSE_SRC})
target_link_libraries(rm_pose opencv_calib3d)