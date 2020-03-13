# compile source files in this directory into a static library
# named rm_armor
AUX_SOURCE_DIRECTORY(. ARMOR_SRC)
ADD_LIBRARY(rm_armor ${ARMOR_SRC})