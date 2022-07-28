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
find_package(Boost 1.50 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

# GPMP2
find_package( Eigen3 REQUIRED
              gpmp2 REQUIRED)

set(GPMP2_LIBRARIES gpmp2)
# ----------------------- duplicated contents from the higher level CMakeLists ------------------------

set(vimp_subdirs 
    optimizer
    helpers 
    instances
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


# add_executable(main src/main.cpp)
# target_link_libraries(main ${PYTHON_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(conv src/conv.cpp)
target_link_libraries(conv  ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(factor_Gaussian_GH src/conv_factorized_GH.cpp)
target_link_libraries(factor_Gaussian_GH ${EIGEN3_LIBRARIES})

add_executable(factor src/conv_factorized.cpp)
target_link_libraries(factor  ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(sparse_inverse src/sparse_inverse.cpp)
target_link_libraries(sparse_inverse ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(Gaussian_GH src/conv_Gaussian_GH.cpp)
target_link_libraries(Gaussian_GH ${EIGEN3_LIBRARIES})

add_executable(prior_col_pR src/conv_prior_col_pR.cpp)
target_link_libraries(prior_col_pR ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(prior_col_pR_cython src/conv_prior_col_pR_cython.cpp)
target_link_libraries(prior_col_pR_cython ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(prior_pR src/conv_prior_pR.cpp)
target_link_libraries(prior_pR ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(prior_pR_multiobs src/conv_prior_col_pR_multiobs.cpp)
target_link_libraries(prior_pR_multiobs ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(conv_1d src/conv_1d_example.cpp)
target_link_libraries(conv_1d ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})

# add_executable(conv_prior_col_pR_gp_inter src/conv_prior_col_pR_gp_inter.cpp)
# target_link_libraries(conv_prior_col_pR_gp_inter ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})


# -------------------------------------------- Google tests
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)


#% --------------------------------------------------------- Tests files 
add_executable(test_inverser tests/test_inverser.cpp)
target_link_libraries(test_inverser ${EIGEN3_LIBRARIES} GTest::gtest_main)

add_executable(test_optimizer tests/test_optimizer.cpp)
target_link_libraries(test_optimizer ${EIGEN3_LIBRARIES} GTest::gtest_main)

add_executable(test_cython tests/test_cython.cpp)
target_link_libraries(test_cython ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES} ${EIGEN3_LIBRARIES} GTest::gtest_main)

add_executable(test_Gh_instance instances/tests/test_GHInstance.cpp)
target_link_libraries(test_Gh_instance ${EIGEN3_LIBRARIES} GTest::gtest_main)

add_executable(test_GH tests/test_GH.cpp)
target_link_libraries(test_GH ${EIGEN3_LIBRARIES} GTest::gtest_main)

add_executable(test_gaussian tests/test_gaussian.cpp)
target_link_libraries(test_gaussian ${EIGEN3_LIBRARIES} GTest::gtest_main)

add_executable(test_optimizer_functionals tests/test_optimizer_functionals.cpp)
target_link_libraries(test_optimizer_functionals ${EIGEN3_LIBRARIES} GTest::gtest_main)

add_executable(test_read_sdf tests/test_read_sdf.cpp)
target_link_libraries(test_read_sdf ${EIGEN3_LIBRARIES} GTest::gtest_main)

# -------------------------------------------- Doxygen for documentations
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/doxygen/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        COMMAND firefox ${CMAKE_CURRENT_SOURCE_DIR}/html/index.html
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
        
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)


