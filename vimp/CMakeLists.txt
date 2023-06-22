# ----------------------- duplicated contents from the higher level CMakeLists ------------------------
cmake_minimum_required(VERSION 3.0)
enable_testing()
project(vimp CXX C)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
    set(CMAKE_MACOSX_RPATH 0)
endif()

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# GPMP2
find_package( Eigen3 3.4.0 REQUIRED
              gpmp2 REQUIRED)

set(GPMP2_LIBRARIES gpmp2)

# matplotcpp
find_package(Matplot++ REQUIRED)

# adept
find_package(FastAD CONFIG REQUIRED)

# argarse
find_package(argparse REQUIRED)

# ----------------------- duplicated contents from the higher level CMakeLists ------------------------

set(vimp_subdirs 
    base
    optimizer
    helpers 
    instances
    robots
    covariance_steering
    dynamics
)

set(vimp_srcs)

# files want to be excluded
set(excluded_sources "helpers/matplotlibcpp.h" "helpers/plotPlanarResults.h")

# Library sources
foreach(subdir ${vimp_subdirs})
  file(GLOB subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h")
  list(REMOVE_ITEM subdir_srcs "${excluded_sources}")

  # message(${subdir_srcs})
  file(GLOB subdir_test_files "${subdir}/tests/*")

  list(APPEND vimp_srcs ${subdir_srcs})
  message(STATUS "Building Module: ${subdir}")

  # local and tests
  add_subdirectory(${subdir})
  
endforeach(subdir)

# include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${GPMP2_INCLUDE_DIRS})
include_directories(${GTSAM_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${ARGPARSE_INCLUDE_DIRS})

# Add all the source files in src/vimp as a separate executable
file(GLOB_RECURSE VIMP_SOURCES src/vimp/*.cpp)
foreach(vimp_source ${VIMP_SOURCES})
    get_filename_component(vimp_name ${vimp_source} NAME_WE)
    add_executable(${vimp_name} ${vimp_source})
    set_target_properties(${vimp_name} PROPERTIES LINKER_LANGUAGE CXX)
    target_link_libraries(${vimp_name} ${EIGEN3_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES})
endforeach()

# Add all the source files in src/pgcs as a separate executable
file(GLOB_RECURSE PGCS_SOURCES src/pgcs/*.cpp)
foreach(pgcs_source ${PGCS_SOURCES})
    get_filename_component(pgcs_name ${pgcs_source} NAME_WE)
    add_executable(${pgcs_name} ${pgcs_source})
    set_target_properties(${pgcs_name} PROPERTIES LINKER_LANGUAGE CXX)
    target_link_libraries(${pgcs_name} ${EIGEN3_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES} Matplot++::matplot)
    target_include_directories(${pgcs_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

#% --------------------------------------------------------- Tests files 
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Recursively search for all test files in the subfolders
file(GLOB_RECURSE TEST_SOURCES vimp_subdirs false */tests/*.cpp)
  
# Loop through each test file and add it as a test case
# foreach(TEST_SOURCE ${TEST_SOURCES})
#     get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
#     add_executable(${TEST_NAME} ${TEST_SOURCE})
#     target_link_libraries(${TEST_NAME} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} Matplot++::matplot FastAD::FastAD ${GPMP2_LIBRARIES} ${GTSAM_LIBRARIES})
#     # add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
#     # target_include_directories(${TEST_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# endforeach()

# -------------------------------------------- Doxygen for documentations
# find_package(Doxygen)
# if (DOXYGEN_FOUND)
#     # set input and output files
#     set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
#     set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/doxygen/Doxyfile)

#     # request to configure the file
#     configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
#     message("Doxygen build started")

#     # note the option ALL which allows to build the docs together with the application
#     add_custom_target( doc_doxygen ALL
#         COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
#         COMMAND firefox ${CMAKE_CURRENT_SOURCE_DIR}/html/index.html
#         WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#         COMMENT "Generating API documentation with Doxygen"
#         VERBATIM )
        
# else (DOXYGEN_FOUND)
#   message("Doxygen need to be installed to generate the doxygen documentation")
# endif (DOXYGEN_FOUND)

