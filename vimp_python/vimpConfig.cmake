# - Config file for vimp
# It defines the following variables
#  vimp_INCLUDE_DIR - include directories for vimp
 
# Compute paths
get_filename_component(OUR_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
if(EXISTS "${OUR_CMAKE_DIR}/CMakeCache.txt")
  # In build tree
  set(vimp_INCLUDE_DIR /home/hongzhe/git/VIMP CACHE PATH "vimp include directory")
else()
  # Find installed library
  set(vimp_INCLUDE_DIR "${OUR_CMAKE_DIR}/../../../include" CACHE PATH "vimp include directory")
endif()
  
# Load exports
include(${OUR_CMAKE_DIR}/vimp-exports.cmake)

# Load project-specific flags, if present
if(EXISTS "${OUR_CMAKE_DIR}/_does_not_exist_")
	include("${OUR_CMAKE_DIR}/_does_not_exist_")
endif()

message(STATUS "vimp include directory:  ${vimp_INCLUDE_DIR}")
