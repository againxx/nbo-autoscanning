###############################################################################
# Find OpenNI2
#
# This sets the following variables:
# OPENNI2_FOUND - True if OPENNI was found.
# OPENNI2_INCLUDE_DIRS - Directories containing the OPENNI include files.
# OPENNI2_LIBRARIES - Libraries needed to use OPENNI.


find_path(OPENNI2_INCLUDE_DIR OpenNI.h
          PATHS
            "${CMAKE_SOURCE_DIR}/../OpenNI2/Include"
            "${CMAKE_SOURCE_DIR}/../../OpenNI2/Include"
            "${CMAKE_SOURCE_DIR}/../../../OpenNI2/Include"
            "${CMAKE_SOURCE_DIR}/../../../../OpenNI2/Include"
            "${CMAKE_SOURCE_DIR}/../../../code/OpenNI2/Include"
            "${CMAKE_SOURCE_DIR}/../../../../code/OpenNI2/Include"
            "${CMAKE_SOURCE_DIR}/../deps/OpenNI2/Include"
            "${CMAKE_SOURCE_DIR}/../../deps/OpenNI2/Include"
            "${CMAKE_SOURCE_DIR}/../../../deps/OpenNI2/Include"
            "${CMAKE_SOURCE_DIR}/../../../../deps/OpenNI2/Include"
	    "${CMAKE_SOURCE_DIR}/iroboscan/deps/OpenNI2/Include"
          PATH_SUFFIXES openni2 ni2
  	  NO_DEFAULT_PATH
)
message(${CMAKE_SOURCE_DIR})

#add a hint so that it can find it without the pkg-config
find_library(OPENNI2_LIBRARY
             NAMES OpenNI2
             PATHS
               "${CMAKE_SOURCE_DIR}/../OpenNI2/Bin/x64-Release"
               "${CMAKE_SOURCE_DIR}/../../OpenNI2/Bin/x64-Release"
               "${CMAKE_SOURCE_DIR}/../../../OpenNI2/Bin/x64-Release"
               "${CMAKE_SOURCE_DIR}/../../../../OpenNI2/Bin/x64-Release"
               "${CMAKE_SOURCE_DIR}/../../../code/OpenNI2/Bin/x64-Release"
               "${CMAKE_SOURCE_DIR}/../../../../code/OpenNI2/Bin/x64-Release"
               "${CMAKE_SOURCE_DIR}/../deps/OpenNI2/Bin/x64-Release"
               "${CMAKE_SOURCE_DIR}/../../deps/OpenNI2/Bin/x64-Release"
               "${CMAKE_SOURCE_DIR}/../../../deps/OpenNI2/Bin/x64-Release"
               "${CMAKE_SOURCE_DIR}/../../../../deps/OpenNI2/Bin/x64-Release"
	       "${CMAKE_SOURCE_DIR}/iroboscan/deps/OpenNI2/Bin/x64-Release"
             PATH_SUFFIXES ${OPENNI_PATH_SUFFIXES}
	     NO_DEFAULT_PATH
)

set(OPENNI2_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIR})
set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2 DEFAULT_MSG
    OPENNI2_LIBRARY OPENNI2_INCLUDE_DIR)

mark_as_advanced(OPENNI2_LIBRARY OPENNI2_INCLUDE_DIR)
