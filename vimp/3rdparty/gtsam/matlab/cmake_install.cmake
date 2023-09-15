# Install script for directory: /home/hzyu/git/gtsam/matlab

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxDebug/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxDebug" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/[^/]*\\.m$" REGEX "/[^/]*\\.fig$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolbox/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolbox" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/[^/]*\\.m$" REGEX "/[^/]*\\.fig$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Tt][Ii][Mm][Ii][Nn][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxTiming/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxTiming" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/[^/]*\\.m$" REGEX "/[^/]*\\.fig$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Tt][Ii][Mm][Ii][Nn][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Pp][Rr][Oo][Ff][Ii][Ll][Ii][Nn][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxProfiling/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxProfiling" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/[^/]*\\.m$" REGEX "/[^/]*\\.fig$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Pp][Rr][Oo][Ff][Ii][Ll][Ii][Nn][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxRelWithDebInfo/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxRelWithDebInfo" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/[^/]*\\.m$" REGEX "/[^/]*\\.fig$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxMinSizeRel/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxMinSizeRel" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/[^/]*\\.m$" REGEX "/[^/]*\\.fig$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxDebug/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxDebug" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/README\\-gtsam\\-toolbox\\.txt$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolbox/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolbox" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/README\\-gtsam\\-toolbox\\.txt$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Tt][Ii][Mm][Ii][Nn][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxTiming/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxTiming" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/README\\-gtsam\\-toolbox\\.txt$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Tt][Ii][Mm][Ii][Nn][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Pp][Rr][Oo][Ff][Ii][Ll][Ii][Nn][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxProfiling/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxProfiling" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/README\\-gtsam\\-toolbox\\.txt$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Pp][Rr][Oo][Ff][Ii][Ll][Ii][Nn][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxRelWithDebInfo/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxRelWithDebInfo" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/README\\-gtsam\\-toolbox\\.txt$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxMinSizeRel/")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxMinSizeRel" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/matlab/" FILES_MATCHING REGEX "/README\\-gtsam\\-toolbox\\.txt$" REGEX "/$" EXCLUDE)
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/example.graph;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/sphere_smallnoise.graph;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/w100.graph;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/w10000.graph;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/Plaza1_.mat;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/Plaza2_.mat;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/5pointExample1.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/5pointExample2.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/KittiEquivBiasedImu.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/KittiEquivBiasedImu_metadata.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/KittiGps_converted.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/Plaza1_DR.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/Plaza1_TD.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/Plaza2_DR.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/Plaza2_TD.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/VO_calibration.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/VO_calibration00.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/VO_calibration00s.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/VO_camera_poses00.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/VO_camera_poses00s.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/VO_camera_poses_large.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/VO_stereo_factors00.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/VO_stereo_factors00s.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/VO_stereo_factors_large.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/dubrovnik-1-1-pre.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/dubrovnik-3-7-18-pre.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/dubrovnik-3-7-pre.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/noisyToyGraph.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/optimizedNoisyToyGraph.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/orientationsNoisyToyGraph.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/pose2example.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/pose3example-grid.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/pose3example-offdiagonal-rewritten.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/pose3example-offdiagonal.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/pose3example.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/simpleGraph10gradIter.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/sphere2500.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/sphere2500_groundtruth.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/victoria_park.txt;/usr/local/gtsam_toolboxDebug/gtsam_examples/Data/w20000.txt")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxDebug/gtsam_examples/Data" TYPE FILE FILES
      "/home/hzyu/git/gtsam/examples/Data/example.graph"
      "/home/hzyu/git/gtsam/examples/Data/sphere_smallnoise.graph"
      "/home/hzyu/git/gtsam/examples/Data/w100.graph"
      "/home/hzyu/git/gtsam/examples/Data/w10000.graph"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_.mat"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_.mat"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample1.txt"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample2.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu_metadata.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiGps_converted.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-1-1-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-18-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/noisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/optimizedNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/orientationsNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose2example.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-grid.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal-rewritten.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example.txt"
      "/home/hzyu/git/gtsam/examples/Data/simpleGraph10gradIter.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500_groundtruth.txt"
      "/home/hzyu/git/gtsam/examples/Data/victoria_park.txt"
      "/home/hzyu/git/gtsam/examples/Data/w20000.txt"
      )
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolbox/gtsam_examples/Data/example.graph;/usr/local/gtsam_toolbox/gtsam_examples/Data/sphere_smallnoise.graph;/usr/local/gtsam_toolbox/gtsam_examples/Data/w100.graph;/usr/local/gtsam_toolbox/gtsam_examples/Data/w10000.graph;/usr/local/gtsam_toolbox/gtsam_examples/Data/Plaza1_.mat;/usr/local/gtsam_toolbox/gtsam_examples/Data/Plaza2_.mat;/usr/local/gtsam_toolbox/gtsam_examples/Data/5pointExample1.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/5pointExample2.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/KittiEquivBiasedImu.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/KittiEquivBiasedImu_metadata.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/KittiGps_converted.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/Plaza1_DR.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/Plaza1_TD.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/Plaza2_DR.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/Plaza2_TD.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/VO_calibration.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/VO_calibration00.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/VO_calibration00s.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/VO_camera_poses00.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/VO_camera_poses00s.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/VO_camera_poses_large.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/VO_stereo_factors00.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/VO_stereo_factors00s.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/VO_stereo_factors_large.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/dubrovnik-1-1-pre.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/dubrovnik-3-7-18-pre.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/dubrovnik-3-7-pre.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/noisyToyGraph.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/optimizedNoisyToyGraph.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/orientationsNoisyToyGraph.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/pose2example.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/pose3example-grid.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/pose3example-offdiagonal-rewritten.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/pose3example-offdiagonal.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/pose3example.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/simpleGraph10gradIter.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/sphere2500.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/sphere2500_groundtruth.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/victoria_park.txt;/usr/local/gtsam_toolbox/gtsam_examples/Data/w20000.txt")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolbox/gtsam_examples/Data" TYPE FILE FILES
      "/home/hzyu/git/gtsam/examples/Data/example.graph"
      "/home/hzyu/git/gtsam/examples/Data/sphere_smallnoise.graph"
      "/home/hzyu/git/gtsam/examples/Data/w100.graph"
      "/home/hzyu/git/gtsam/examples/Data/w10000.graph"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_.mat"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_.mat"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample1.txt"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample2.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu_metadata.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiGps_converted.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-1-1-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-18-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/noisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/optimizedNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/orientationsNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose2example.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-grid.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal-rewritten.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example.txt"
      "/home/hzyu/git/gtsam/examples/Data/simpleGraph10gradIter.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500_groundtruth.txt"
      "/home/hzyu/git/gtsam/examples/Data/victoria_park.txt"
      "/home/hzyu/git/gtsam/examples/Data/w20000.txt"
      )
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Tt][Ii][Mm][Ii][Nn][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/example.graph;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/sphere_smallnoise.graph;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/w100.graph;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/w10000.graph;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/Plaza1_.mat;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/Plaza2_.mat;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/5pointExample1.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/5pointExample2.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/KittiEquivBiasedImu.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/KittiEquivBiasedImu_metadata.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/KittiGps_converted.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/Plaza1_DR.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/Plaza1_TD.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/Plaza2_DR.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/Plaza2_TD.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/VO_calibration.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/VO_calibration00.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/VO_calibration00s.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/VO_camera_poses00.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/VO_camera_poses00s.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/VO_camera_poses_large.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/VO_stereo_factors00.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/VO_stereo_factors00s.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/VO_stereo_factors_large.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/dubrovnik-1-1-pre.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/dubrovnik-3-7-18-pre.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/dubrovnik-3-7-pre.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/noisyToyGraph.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/optimizedNoisyToyGraph.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/orientationsNoisyToyGraph.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/pose2example.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/pose3example-grid.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/pose3example-offdiagonal-rewritten.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/pose3example-offdiagonal.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/pose3example.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/simpleGraph10gradIter.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/sphere2500.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/sphere2500_groundtruth.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/victoria_park.txt;/usr/local/gtsam_toolboxTiming/gtsam_examples/Data/w20000.txt")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxTiming/gtsam_examples/Data" TYPE FILE FILES
      "/home/hzyu/git/gtsam/examples/Data/example.graph"
      "/home/hzyu/git/gtsam/examples/Data/sphere_smallnoise.graph"
      "/home/hzyu/git/gtsam/examples/Data/w100.graph"
      "/home/hzyu/git/gtsam/examples/Data/w10000.graph"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_.mat"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_.mat"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample1.txt"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample2.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu_metadata.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiGps_converted.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-1-1-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-18-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/noisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/optimizedNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/orientationsNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose2example.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-grid.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal-rewritten.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example.txt"
      "/home/hzyu/git/gtsam/examples/Data/simpleGraph10gradIter.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500_groundtruth.txt"
      "/home/hzyu/git/gtsam/examples/Data/victoria_park.txt"
      "/home/hzyu/git/gtsam/examples/Data/w20000.txt"
      )
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Tt][Ii][Mm][Ii][Nn][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Pp][Rr][Oo][Ff][Ii][Ll][Ii][Nn][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/example.graph;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/sphere_smallnoise.graph;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/w100.graph;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/w10000.graph;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/Plaza1_.mat;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/Plaza2_.mat;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/5pointExample1.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/5pointExample2.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/KittiEquivBiasedImu.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/KittiEquivBiasedImu_metadata.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/KittiGps_converted.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/Plaza1_DR.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/Plaza1_TD.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/Plaza2_DR.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/Plaza2_TD.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/VO_calibration.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/VO_calibration00.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/VO_calibration00s.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/VO_camera_poses00.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/VO_camera_poses00s.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/VO_camera_poses_large.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/VO_stereo_factors00.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/VO_stereo_factors00s.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/VO_stereo_factors_large.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/dubrovnik-1-1-pre.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/dubrovnik-3-7-18-pre.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/dubrovnik-3-7-pre.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/noisyToyGraph.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/optimizedNoisyToyGraph.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/orientationsNoisyToyGraph.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/pose2example.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/pose3example-grid.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/pose3example-offdiagonal-rewritten.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/pose3example-offdiagonal.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/pose3example.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/simpleGraph10gradIter.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/sphere2500.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/sphere2500_groundtruth.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/victoria_park.txt;/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data/w20000.txt")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxProfiling/gtsam_examples/Data" TYPE FILE FILES
      "/home/hzyu/git/gtsam/examples/Data/example.graph"
      "/home/hzyu/git/gtsam/examples/Data/sphere_smallnoise.graph"
      "/home/hzyu/git/gtsam/examples/Data/w100.graph"
      "/home/hzyu/git/gtsam/examples/Data/w10000.graph"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_.mat"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_.mat"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample1.txt"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample2.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu_metadata.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiGps_converted.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-1-1-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-18-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/noisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/optimizedNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/orientationsNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose2example.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-grid.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal-rewritten.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example.txt"
      "/home/hzyu/git/gtsam/examples/Data/simpleGraph10gradIter.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500_groundtruth.txt"
      "/home/hzyu/git/gtsam/examples/Data/victoria_park.txt"
      "/home/hzyu/git/gtsam/examples/Data/w20000.txt"
      )
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Pp][Rr][Oo][Ff][Ii][Ll][Ii][Nn][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/example.graph;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/sphere_smallnoise.graph;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/w100.graph;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/w10000.graph;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/Plaza1_.mat;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/Plaza2_.mat;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/5pointExample1.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/5pointExample2.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/KittiEquivBiasedImu.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/KittiEquivBiasedImu_metadata.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/KittiGps_converted.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/Plaza1_DR.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/Plaza1_TD.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/Plaza2_DR.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/Plaza2_TD.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/VO_calibration.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/VO_calibration00.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/VO_calibration00s.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/VO_camera_poses00.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/VO_camera_poses00s.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/VO_camera_poses_large.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/VO_stereo_factors00.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/VO_stereo_factors00s.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/VO_stereo_factors_large.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/dubrovnik-1-1-pre.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/dubrovnik-3-7-18-pre.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/dubrovnik-3-7-pre.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/noisyToyGraph.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/optimizedNoisyToyGraph.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/orientationsNoisyToyGraph.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/pose2example.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/pose3example-grid.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/pose3example-offdiagonal-rewritten.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/pose3example-offdiagonal.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/pose3example.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/simpleGraph10gradIter.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/sphere2500.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/sphere2500_groundtruth.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/victoria_park.txt;/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data/w20000.txt")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_examples/Data" TYPE FILE FILES
      "/home/hzyu/git/gtsam/examples/Data/example.graph"
      "/home/hzyu/git/gtsam/examples/Data/sphere_smallnoise.graph"
      "/home/hzyu/git/gtsam/examples/Data/w100.graph"
      "/home/hzyu/git/gtsam/examples/Data/w10000.graph"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_.mat"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_.mat"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample1.txt"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample2.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu_metadata.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiGps_converted.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-1-1-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-18-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/noisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/optimizedNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/orientationsNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose2example.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-grid.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal-rewritten.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example.txt"
      "/home/hzyu/git/gtsam/examples/Data/simpleGraph10gradIter.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500_groundtruth.txt"
      "/home/hzyu/git/gtsam/examples/Data/victoria_park.txt"
      "/home/hzyu/git/gtsam/examples/Data/w20000.txt"
      )
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/example.graph;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/sphere_smallnoise.graph;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/w100.graph;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/w10000.graph;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/Plaza1_.mat;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/Plaza2_.mat;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/5pointExample1.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/5pointExample2.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/KittiEquivBiasedImu.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/KittiEquivBiasedImu_metadata.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/KittiGps_converted.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/Plaza1_DR.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/Plaza1_TD.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/Plaza2_DR.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/Plaza2_TD.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/VO_calibration.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/VO_calibration00.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/VO_calibration00s.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/VO_camera_poses00.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/VO_camera_poses00s.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/VO_camera_poses_large.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/VO_stereo_factors00.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/VO_stereo_factors00s.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/VO_stereo_factors_large.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/dubrovnik-1-1-pre.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/dubrovnik-3-7-18-pre.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/dubrovnik-3-7-pre.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/noisyToyGraph.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/optimizedNoisyToyGraph.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/orientationsNoisyToyGraph.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/pose2example.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/pose3example-grid.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/pose3example-offdiagonal-rewritten.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/pose3example-offdiagonal.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/pose3example.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/simpleGraph10gradIter.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/sphere2500.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/sphere2500_groundtruth.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/victoria_park.txt;/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data/w20000.txt")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxMinSizeRel/gtsam_examples/Data" TYPE FILE FILES
      "/home/hzyu/git/gtsam/examples/Data/example.graph"
      "/home/hzyu/git/gtsam/examples/Data/sphere_smallnoise.graph"
      "/home/hzyu/git/gtsam/examples/Data/w100.graph"
      "/home/hzyu/git/gtsam/examples/Data/w10000.graph"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_.mat"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_.mat"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample1.txt"
      "/home/hzyu/git/gtsam/examples/Data/5pointExample2.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiEquivBiasedImu_metadata.txt"
      "/home/hzyu/git/gtsam/examples/Data/KittiGps_converted.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza1_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_DR.txt"
      "/home/hzyu/git/gtsam/examples/Data/Plaza2_TD.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_calibration00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_camera_poses_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors00s.txt"
      "/home/hzyu/git/gtsam/examples/Data/VO_stereo_factors_large.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-1-1-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-18-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/dubrovnik-3-7-pre.txt"
      "/home/hzyu/git/gtsam/examples/Data/noisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/optimizedNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/orientationsNoisyToyGraph.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose2example.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-grid.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal-rewritten.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example-offdiagonal.txt"
      "/home/hzyu/git/gtsam/examples/Data/pose3example.txt"
      "/home/hzyu/git/gtsam/examples/Data/simpleGraph10gradIter.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500.txt"
      "/home/hzyu/git/gtsam/examples/Data/sphere2500_groundtruth.txt"
      "/home/hzyu/git/gtsam/examples/Data/victoria_park.txt"
      "/home/hzyu/git/gtsam/examples/Data/w20000.txt"
      )
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
endif()

