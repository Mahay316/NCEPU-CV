# compile source files in this directory into a static library
# named rm_pose
AUX_SOURCE_DIRECTORY(. POSE_SRC)
ADD_LIBRARY(rm_pose ${POSE_SRC})