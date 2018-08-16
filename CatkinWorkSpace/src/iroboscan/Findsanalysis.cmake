###############################################################################
# Find sanalysis
#
# This sets the following variables:
# SANALYSIS_FOUND - True if SANALYSIS was found.
# SANALYSIS_INCLUDE_DIRS - Directories containing the SANALYSIS include files.
# SANALYSIS_LIBRARIES - Libraries needed to use SANALYSIS .

find_package(PCL 1.3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin 0.1 REQUIRED)

find_path(SANALYSIS_INCLUDE_DIR
		  NAMES ScanningScheme.h
          PATHS ~/iroboscan/Seg/src
		  PATH_SUFFIXES Seg
)

find_library(SANALYSIS_LIBRARY
             NAMES sanalysis
             PATHS ~/iroboscan/Seg/src/build
			 PATH_SUFFIXES ${SANALYSIS_PATH_SUFFIXES}
)

set(SANALYSIS_INCLUDE_DIRS ${SANALYSIS_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
set(SANALYSIS_LIBRARIES ${SANALYSIS_LIBRARY} ${PCL_LIBRARIES} ${CUDA_LIBRARIES} ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SANALYSIS DEFAULT_MSG SANALYSIS_LIBRARY SANALYSIS_INCLUDE_DIR)
mark_as_advanced(SANALYSIS_LIBRARY SANALYSIS_INCLUDE_DIR)
