###############################################################################
# Find darknet
#
# This sets the following variables:
# DARKNET_FOUND - True if DARKNET was found.
# DARKNET_INCLUDE_DIRS - Directories containing the DARKNET include files.
# DARKNET_LIBRARIES - Libraries needed to use DARKNET .

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

find_path(DARKNET_INCLUDE_DIR
		  NAMES darknet.c
          PATHS ~/iroboscan/darknet/src
		  PATH_SUFFIXES darknet
)

find_library(DARKNET_LIBRARY
             NAMES darknet-cpp-shared
             PATHS ~/iroboscan/darknet
			 PATH_SUFFIXES ${DARKNET_PATH_SUFFIXES}
)

set(DARKNET_INCLUDE_DIRS ${DARKNET_INCLUDE_DIR} ${CUDA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(DARKNET_LIBRARIES ${DARKNET_LIBRARY} ${CUDA_LIBRARIES} ${OpenCV_LIBS})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DARKNET DEFAULT_MSG DARKNET_LIBRARY DARKNET_INCLUDE_DIR)
mark_as_advanced(DARKNET_LIBRARY DARKNET_INCLUDE_DIR)
