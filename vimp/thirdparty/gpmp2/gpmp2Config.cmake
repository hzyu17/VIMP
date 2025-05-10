# - Config file for gpmp2
# It defines the following variables
#  gpmp2_INCLUDE_DIR - include directories for gpmp2
 
# Compute paths
get_filename_component(OUR_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
if(EXISTS "${OUR_CMAKE_DIR}/CMakeCache.txt")
  # In build tree
  set(gpmp2_INCLUDE_DIR /home/hzyu/git/gpmp2 CACHE PATH "gpmp2 include directory")
else()
  # Find installed library
  set(gpmp2_INCLUDE_DIR "${OUR_CMAKE_DIR}/../../../include" CACHE PATH "gpmp2 include directory")
endif()
  
# Load exports
include(${OUR_CMAKE_DIR}/gpmp2-exports.cmake)

# Load project-specific flags, if present
if(EXISTS "${OUR_CMAKE_DIR}/_does_not_exist_")
	include("${OUR_CMAKE_DIR}/_does_not_exist_")
endif()

message(STATUS "gpmp2 include directory:  ${gpmp2_INCLUDE_DIR}")
