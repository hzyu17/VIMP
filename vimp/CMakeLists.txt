message("Building submodule: vimp")
# ------------- submodules -------------
set(vimp_subdirs 
    GaussianVI
    pgcsmp
    helpers 
    instances
    robots
    dynamics
)

set(vimp_srcs)

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

# ----------------------- options -----------------------
# Add a flag to control whether to build CPU executables
option(BUILD_CPU_GVIMP "Build CPU version GVIMP executables from src/gvimp" OFF)
option(BUILD_GPU_GVIMP "Build GPU version GVIMP executables from src/gvimp_cuda" ON)
option(BUILD_KL_PROXIMAL "Build KL-Proximal executables from src/proxkl" OFF)

# ---------------------- CPU Executables (Optional) ----------------------
if(BUILD_CPU_GVIMP)
  file(GLOB_RECURSE VIMP_SOURCES src/gvimp/*.cpp)
  foreach(vimp_source ${VIMP_SOURCES})
      get_filename_component(vimp_name ${vimp_source} NAME_WE)
      add_executable(${vimp_name} ${vimp_source})
      set_target_properties(${vimp_name} PROPERTIES LINKER_LANGUAGE CXX)
      set_target_properties(${vimp_name} PROPERTIES
                              RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/src/gvimp"
                              )   
      target_include_directories(${vimp_name} PRIVATE ${Matlab_INCLUDE_DIRS})
      target_link_libraries(${vimp_name} ${EIGEN3_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES} 
                            ${Matlab_MEX_LIBRARY} ${Matlab_MX_LIBRARY} OpenMP::OpenMP_CXX
                            ${libSpGH_LIBRARY} ${MCR_LIBRARY_PATH}/libmwmclmcrrt.so.9.9 yaml-cpp)
  endforeach()
endif()

if(BUILD_GPU_GVIMP)
  file(GLOB_RECURSE VIMP_SOURCES src/gvimp_cuda/*.cpp)
  foreach(vimp_source ${VIMP_SOURCES})
      get_filename_component(vimp_name ${vimp_source} NAME_WE)
      add_executable(${vimp_name} ${vimp_source} GaussianVI/helpers/CudaOperation.cu)
      set_target_properties(${vimp_name} PROPERTIES LINKER_LANGUAGE CXX)
      set_target_properties(${vimp_name} PROPERTIES
                              RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/src/gvimp"
                              )   
      target_include_directories(${vimp_name} PRIVATE ${Matlab_INCLUDE_DIRS})
      target_link_libraries(${vimp_name} ${EIGEN3_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES} 
                            ${Matlab_MEX_LIBRARY} ${Matlab_MX_LIBRARY} OpenMP::OpenMP_CXX
                            ${libSpGH_LIBRARY} ${MCR_LIBRARY_PATH}/libmwmclmcrrt.so.9.9 ${GSL_LIBRARIES} 
                            ${CUDA_CUBLAS_LIBRARIES} ${CUDA_cusolver_LIBRARY} ${CUDA_CUDART_LIBRARY} 
                            ${CUDA_cusparse_LIBRARY} yaml-cpp) 
  endforeach()

  # add_executable(Franka src/gvimp_cuda/gvi_FrankaArm_spgh_cuda.cpp GaussianVI/helpers/CudaOperation.cu)
  # set_target_properties(Franka PROPERTIES LINKER_LANGUAGE CXX)
  # set_target_properties(Franka PROPERTIES
  #                         RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/src/gvimp"
  #                         )   
  # target_include_directories(Franka PRIVATE ${Matlab_INCLUDE_DIRS})
  # target_link_libraries(Franka ${EIGEN3_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES} 
  #                           ${Matlab_MEX_LIBRARY} ${Matlab_MX_LIBRARY} OpenMP::OpenMP_CXX
  #                           ${libSpGH_LIBRARY} ${MCR_LIBRARY_PATH}/libmwmclmcrrt.so.9.9 ${GSL_LIBRARIES} 
  #                           ${CUDA_CUBLAS_LIBRARIES} ${CUDA_cusolver_LIBRARY} ${CUDA_CUDART_LIBRARY} 
  #                           ${CUDA_cusparse_LIBRARY}) 
endif()

if(BUILD_KL_PROXIMAL)
  file(GLOB_RECURSE VIMP_SOURCES src/proxkl/*.cpp)
  foreach(vimp_source ${VIMP_SOURCES})
      get_filename_component(vimp_name ${vimp_source} NAME_WE)
      add_executable(${vimp_name} ${vimp_source} GaussianVI/helpers/CudaOperation.cu)
      set_target_properties(${vimp_name} PROPERTIES LINKER_LANGUAGE CXX)
      set_target_properties(${vimp_name} PROPERTIES
                              RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/src/gvimp"
                              )
      target_include_directories(${vimp_name} PRIVATE ${Matlab_INCLUDE_DIRS})
      target_link_libraries(${vimp_name} ${EIGEN3_LIBRARIES} ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES} 
                            ${Matlab_MEX_LIBRARY} ${Matlab_MX_LIBRARY} OpenMP::OpenMP_CXX
                            ${libSpGH_LIBRARY} ${MCR_LIBRARY_PATH}/libmwmclmcrrt.so.9.9 ${GSL_LIBRARIES} 
                            ${CUDA_CUBLAS_LIBRARIES} ${CUDA_cusolver_LIBRARY} ${CUDA_CUDART_LIBRARY} 
                            ${CUDA_cusparse_LIBRARY} yaml-cpp)  
  endforeach()
endif()

pybind11_add_module(bind_Params
    pybinds/bind_params.cpp)

# pybind11_add_module(bind_WamSDF
#     pybinds/bind_WAMSDF.cpp)

# pybind11_add_module(bind_SDF
#     pybinds/bind_sdf.cpp)

pybind11_add_module(bind_FK
    pybinds/bind_fk.cpp)

# # target_link_libraries(bind_WamSDF PRIVATE ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES})


# # Add all the source files in src/pgcs as a separate executable
# file(GLOB_RECURSE PGCS_SOURCES src/pgcs/*.cpp)
# foreach(pgcs_source ${PGCS_SOURCES})
#     get_filename_component(pgcs_name ${pgcs_source} NAME_WE)
#     add_executable(${pgcs_name} ${pgcs_source})
#     set_target_properties(${pgcs_name} PROPERTIES LINKER_LANGUAGE CXX)
#     set_target_properties(${pgcs_name} PROPERTIES
#                             RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/src/pgcs"
#                             )   
    
#     target_link_libraries(${pgcs_name} ${EIGEN3_LIBRARIES} 
#                           ${GTSAM_LIBRARIES} ${GPMP2_LIBRARIES})
#                         #   Matplot++::matplot)
#     target_include_directories(${pgcs_name} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/vimp")
# endforeach()
 
#% --------------------------------------------------------- Tests files ---------------------------------------------------------
# find_package(GTest REQUIRED)
# include_directories(${GTEST_INCLUDE_DIRS})

# # Recursively search for all test files in the subfolders
# file(GLOB_RECURSE TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/vimp false tests/*/*.cpp)
# # Loop through each test file and add it as a test case
# foreach(TEST_SOURCE ${TEST_SOURCES})
#     get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
#     add_executable(${TEST_NAME} ${TEST_SOURCE})
#     set_target_properties(${TEST_NAME} PROPERTIES LINKER_LANGUAGE CXX)
#     set_target_properties(${TEST_NAME} PROPERTIES
#                             RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
#                             )
#     target_link_libraries(${TEST_NAME} ${GTEST_LIBRARIES} ${EIGEN3_LIBRARIES} 
#                           ${GTEST_MAIN_LIBRARIES} ${GPMP2_LIBRARIES} 
#                           ${GTSAM_LIBRARIES} ${Matlab_MEX_LIBRARY} ${Matlab_MX_LIBRARY} 
#                           ${libSpGH_LIBRARY} gtest ${MCR_LIBRARY_PATH}/libmwmclmcrrt.so.9.7 Boost::serialization)
#     gtest_discover_tests(${TEST_NAME})
#     target_include_directories(${TEST_NAME} PRIVATE ${Matlab_INCLUDE_DIRS})
#     # target_include_directories(${TEST_NAME} PRIVATE ${Matlab_INCLUDE_DIRS})
# endforeach()
