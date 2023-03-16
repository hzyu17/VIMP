# ----------------------- duplicated contents from the higher level CMakeLists ------------------------
cmake_minimum_required(VERSION 3.0)
enable_testing()
project(vimp CXX C)
set(CMAKE_CXX_STANDARD 14)
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

# Boost - same requirement as gtsam
# find_package(Boost 1.50 REQUIRED)
# find_package(Boost COMPONENTS filesystem REQUIRED)
# find_package(Boost COMPONENTS system REQUIRED)
# find_package(Boost COMPONENTS thread REQUIRED)
# find_package(Boost COMPONENTS serialization REQUIRED)
# find_package(Boost COMPONENTS date_time REQUIRED)
# find_package(Boost COMPONENTS timer REQUIRED)

# include_directories(${Boost_INCLUDE_DIR})

# GPMP2
find_package( Eigen3 3.4.0 REQUIRED
              gpmp2 REQUIRED)

set(GPMP2_LIBRARIES gpmp2)
# ----------------------- duplicated contents from the higher level CMakeLists ------------------------

set(vimp_subdirs 
    optimizer
    helpers 
    instances
    robots
    linear_CS
)

set(vimp_srcs)

# files want to be excluded
set(excluded_sources "helpers/matplotlibcpp.h" "helpers/plotPlanarResults.h")

# Library sources
foreach(subdir ${vimp_subdirs})
  file(GLOB subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h")
  list(REMOVE_ITEM subdir_srcs "${excluded_sources}")

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

# add executables
file(GLOB vimp_source_files RELATIVE "${CMAKE_SOURCE_DIR}/src" "${CMAKE_SOURCE_DIR}/src/*.cpp")

foreach(src_file ${vimp_source_files})
  string( REPLACE ".cpp" "" src_file_prefix ${src_file} )
  add_executable(${src_file_prefix} "src/${src_file}")
  set_target_properties(${src_file_prefix} PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(${src_file_prefix} ${EIGEN3_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES} TinyAD)
endforeach(src_file)


# # -------------------------------------------- TinyAD
include(FetchContent)
FetchContent_Declare(
  tinyad
  GIT_REPOSITORY https://github.com/patr-schm/tinyad.git
  GIT_TAG 75093e14ef0d7bb39657c5f3b2aba1251afaa38c
)

FetchContent_GetProperties(tinyad)
if(NOT tinyad_POPULATED)
  # Fetch the content using previously declared details
  FetchContent_Populate(tinyad)
  message(STATUS "tinyad_SOURCE_DIR: ${tinyad_SOURCE_DIR}")
  message(STATUS "tinyad_BINARY_DIR: ${tinyad_BINARY_DIR}")
  add_subdirectory(${tinyad_SOURCE_DIR} ${tinyad_BINARY_DIR})
endif()

# # -------------------------------------------- Google tests
# include(FetchContent)
# FetchContent_Declare(
#   googletest
#   GIT_REPOSITORY https://github.com/google/googletest.git
#   GIT_TAG release-1.12.1
# )
# # For Windows: Prevent overriding the parent project's compiler/linker settings
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
# FetchContent_MakeAvailable(googletest)
# add_library(gtest STATIC
#   3rd-part/googletest/1.10/googletest/include/gtest/gtest.h
#   3rd-part/googletest/1.10/googletest/src/gtest-all.cc
# )
# target_include_directories(gtest PUBLIC
#   3rd-part/googletest/1.10/googletest/include
# )
# target_link_libraries(gtest Threads::Threads)



#% --------------------------------------------------------- Tests files 
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test_eigen_sparse tests/test_eigen_sparse.cpp)
target_link_libraries(test_eigen_sparse ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

add_executable(test_linear_system tests/test_linear_system.cpp)
target_link_libraries(test_linear_system ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

add_executable(test_linear_CS tests/test_linear_CS.cpp)
target_link_libraries(test_linear_CS ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

add_executable(test_SDF_Hessian tests/test_SDF_Hessian.cpp)
target_link_libraries(test_SDF_Hessian ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES} TinyAD)

# add_executable(test_GH tests/test_GH.cpp)
# target_link_libraries(test_GH ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})

# # add executables for the tests
# file(GLOB vimp_test_files RELATIVE "${CMAKE_SOURCE_DIR}/tests" "${CMAKE_SOURCE_DIR}/tests/*.cpp")

# foreach(test_file ${vimp_test_files})
#   string( REPLACE ".cpp" "" test_file_prefix ${test_file} )
#   add_executable(${test_file_prefix} "tests/${test_file}")
#   set_target_properties(${test_file_prefix} PROPERTIES LINKER_LANGUAGE CXX)
#   target_link_libraries(${src_file_prefix} ${EIGEN3_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES} ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
# endforeach(test_file)


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

