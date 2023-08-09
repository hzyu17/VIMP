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

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

set(BOOST_ROOT "/usr/local/bin/boost")
list(APPEND CMAKE_PREFIX_PATH "${BOOST_ROOT}")
# Boost - same requirement as gtsam
find_package(Boost 1.70 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED)
find_package(Boost COMPONENTS date_time REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# GPMP2 
find_package( Eigen3 3.4.0 REQUIRED
              gpmp2 REQUIRED)

set(GPMP2_LIBRARIES "/usr/local/lib/libgpmp2.so")
set(GTSAM_LIBRARIES "/usr/local/lib/libgtsam.so.4.0.0")
 
# matplotcpp
include(FetchContent)
FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master) # or whatever tag you want

FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# ----------------------- duplicated contents from the higher level CMakeLists ------------------------

set(vimp_subdirs 
    gvimp
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
include_directories(${gpmp2_INCLUDE_DIRS})
include_directories(${gtsam_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_compile_definitions("SOURCE_ROOT=${CMAKE_CURRENT_SOURCE_DIR}")

# Add all the source files in src/vimp as a separate executable
file(GLOB_RECURSE VIMP_SOURCES src/gvimp/*.cpp)
foreach(vimp_source ${VIMP_SOURCES})
    get_filename_component(vimp_name ${vimp_source} NAME_WE)
    add_executable(${vimp_name} ${vimp_source})
    set_target_properties(${vimp_name} PROPERTIES LINKER_LANGUAGE CXX)
    set_target_properties(${vimp_name} PROPERTIES
                            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/src/gvimp"
                            )   
    target_link_libraries(${vimp_name} ${EIGEN3_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES} ${BOOST_LIBRARIES})
endforeach()

# Add all the source files in src/pgcs as a separate executable
file(GLOB_RECURSE PGCS_SOURCES src/pgcs/*.cpp)
foreach(pgcs_source ${PGCS_SOURCES})
    get_filename_component(pgcs_name ${pgcs_source} NAME_WE)
    add_executable(${pgcs_name} ${pgcs_source})
    set_target_properties(${pgcs_name} PROPERTIES LINKER_LANGUAGE CXX)
    set_target_properties(${pgcs_name} PROPERTIES
                            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/src/pgcs"
                            )   
    target_link_libraries(${pgcs_name} ${EIGEN3_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES} Matplot++::matplot)
    target_include_directories(${pgcs_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()
 
#% --------------------------------------------------------- Tests files 
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Recursively search for all test files in the subfolders
file(GLOB_RECURSE TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR} false tests/*/*.cpp)
# Loop through each test file and add it as a test case
foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    set_target_properties(${TEST_NAME} PROPERTIES LINKER_LANGUAGE CXX)
    set_target_properties(${TEST_NAME} PROPERTIES
                            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
                            )
    target_link_libraries(${TEST_NAME} ${GTEST_LIBRARIES} ${EIGEN3_LIBRARIES} ${GTEST_MAIN_LIBRARIES} Matplot++::matplot ${GPMP2_LIBRARIES} ${GTSAM_LIBRARIES})
endforeach()

# # Register the tests with CTest
    # gtest_discover_tests(${TEST_NAME})

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

