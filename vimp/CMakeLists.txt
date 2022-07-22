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

# find_package( PythonLibs REQUIRED)

# include_directories(include)

# include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${GPMP2_INCLUDE_DIRS})
include_directories(${GTSAM_INCLUDE_DIRS})

# build shared lib
if(VIMP_BUILD_STATIC_LIBRARY)
  # static
  message(STATUS "Build static library")
  add_library(${PROJECT_NAME} STATIC ${vimp_srcs})
  set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES})
  set_target_properties(vimp PROPERTIES
      OUTPUT_NAME         vimp
      CLEAN_DIRECT_OUTPUT 1
      VERSION             ${vimp_version}
      SOVERSION           ${vimp_soversion})
  if(WIN32) # Add 'lib' prefix to static library to avoid filename collision with shared library
    set_target_properties(vimp PROPERTIES
      PREFIX "lib")
  endif()
  install(TARGETS ${PROJECT_NAME} EXPORT vimp-exports ARCHIVE DESTINATION lib)

else()
  # shared
  message(STATUS "Build shared library")
  add_library(${PROJECT_NAME} SHARED ${vimp_srcs})
  set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES})
  set_target_properties(vimp PROPERTIES
      OUTPUT_NAME         vimp
      CLEAN_DIRECT_OUTPUT 1
      VERSION             ${vimp_version}
      SOVERSION           ${vimp_soversion})
  if(WIN32)
    set_target_properties(vimp PROPERTIES
        PREFIX ""
        DEFINE_SYMBOL VIMP_EXPORTS
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
  endif()
  install(TARGETS ${PROJECT_NAME} EXPORT vimp-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

endif()

# add_executable(main src/main.cpp)
# target_link_libraries(main ${PYTHON_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(conv src/conv.cpp)
target_link_libraries(conv  ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(conv_factor_Gaussian_GH src/conv_factorized_GH.cpp)
target_link_libraries(conv_factor_Gaussian_GH ${EIGEN3_LIBRARIES})

add_executable(conv_factor src/conv_factorized.cpp)
target_link_libraries(conv_factor  ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(sparse_inverse src/sparse_inverse.cpp)
target_link_libraries(sparse_inverse ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES})

# add_executable(GH src/GH.cpp)
# target_link_libraries(GH ${EIGEN3_LIBRARIES})

add_executable(conv_Gaussian_GH src/conv_Gaussian_GH.cpp)
target_link_libraries(conv_Gaussian_GH ${EIGEN3_LIBRARIES})

add_executable(prior_col_pR src/conv_prior_col_pR.cpp)
target_link_libraries(prior_col_pR ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})

add_executable(prior_pR src/conv_prior_pR.cpp)
target_link_libraries(prior_pR ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})

# add_executable(conv_prior_col_pR_gp_inter src/conv_prior_col_pR_gp_inter.cpp)
# target_link_libraries(conv_prior_col_pR_gp_inter ${GTSAM_LIBRARIES} ${EIGEN3_LIBRARIES} ${GPMP2_LIBRARIES})


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


# #% --------------------------------------------------------- Tests files 
# add_executable(test_inverser tests/test_inverser.cpp)
# target_link_libraries(test_inverser ${EIGEN3_LIBRARIES} GTest::gtest_main)

# add_executable(test_optimizer tests/test_optimizer.cpp)
# target_link_libraries(test_optimizer ${EIGEN3_LIBRARIES} GTest::gtest_main)


# # -------------------------------------------- Doxygen for documentations
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


