# Install script for directory: /home/hzyu/git/gtsam/gtsam

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam" TYPE FILE FILES "/home/hzyu/git/gtsam/gtsam/global_includes.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam" TYPE FILE FILES
    "/home/hzyu/git/gtsam/build/gtsam/config.h"
    "/home/hzyu/git/gtsam/build/gtsam/dllexport.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/hzyu/git/gtsam/build/gtsam/libgtsam.so.4.0.0"
    "/home/hzyu/git/gtsam/build/gtsam/libgtsam.so.4"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/hzyu/git/gtsam/build/gtsam/3rdparty/metis/libmetis:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/hzyu/git/gtsam/build/gtsam/libgtsam.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so"
         OLD_RPATH "/home/hzyu/git/gtsam/build/gtsam/3rdparty/metis/libmetis:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so")
    endif()
  endif()
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxDebug" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/build/wrap/gtsam/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gtsam_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gtsam_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxDebug/gtsam_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxDebug" TYPE MODULE FILES "/home/hzyu/git/gtsam/build/wrap/gtsam_mex/gtsam_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gtsam_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gtsam_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gtsam/build/gtsam:/home/hzyu/git/gtsam/build/gtsam/3rdparty/metis/libmetis:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gtsam_wrapper.mexa64")
      endif()
    endif()
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolbox" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/build/wrap/gtsam/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gtsam_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gtsam_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolbox/gtsam_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolbox" TYPE MODULE FILES "/home/hzyu/git/gtsam/build/wrap/gtsam_mex/gtsam_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gtsam_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gtsam_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gtsam/build/gtsam:/home/hzyu/git/gtsam/build/gtsam/3rdparty/metis/libmetis:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gtsam_wrapper.mexa64")
      endif()
    endif()
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxTiming" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/build/wrap/gtsam/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Tt][Ii][Mm][Ii][Nn][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Tt][Ii][Mm][Ii][Nn][Gg])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gtsam_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gtsam_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxTiming/gtsam_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxTiming" TYPE MODULE FILES "/home/hzyu/git/gtsam/build/wrap/gtsam_mex/gtsam_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gtsam_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gtsam_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gtsam/build/gtsam:/home/hzyu/git/gtsam/build/gtsam/3rdparty/metis/libmetis:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gtsam_wrapper.mexa64")
      endif()
    endif()
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxProfiling" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/build/wrap/gtsam/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Pp][Rr][Oo][Ff][Ii][Ll][Ii][Nn][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Pp][Rr][Oo][Ff][Ii][Ll][Ii][Nn][Gg])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gtsam_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gtsam_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxProfiling/gtsam_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxProfiling" TYPE MODULE FILES "/home/hzyu/git/gtsam/build/wrap/gtsam_mex/gtsam_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gtsam_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gtsam_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gtsam/build/gtsam:/home/hzyu/git/gtsam/build/gtsam/3rdparty/metis/libmetis:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gtsam_wrapper.mexa64")
      endif()
    endif()
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxRelWithDebInfo" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/build/wrap/gtsam/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxRelWithDebInfo" TYPE MODULE FILES "/home/hzyu/git/gtsam/build/wrap/gtsam_mex/gtsam_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gtsam/build/gtsam:/home/hzyu/git/gtsam/build/gtsam/3rdparty/metis/libmetis:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gtsam_wrapper.mexa64")
      endif()
    endif()
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxMinSizeRel" TYPE DIRECTORY FILES "/home/hzyu/git/gtsam/build/wrap/gtsam/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gtsam_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gtsam_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxMinSizeRel/gtsam_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxMinSizeRel" TYPE MODULE FILES "/home/hzyu/git/gtsam/build/wrap/gtsam_mex/gtsam_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gtsam_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gtsam_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gtsam_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gtsam/build/gtsam:/home/hzyu/git/gtsam/build/gtsam/3rdparty/metis/libmetis:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gtsam_wrapper.mexa64")
      endif()
    endif()
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hzyu/git/gtsam/build/gtsam/3rdparty/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/base/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/geometry/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/inference/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/symbolic/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/discrete/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/linear/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/nonlinear/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/sam/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/sfm/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/slam/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/smart/cmake_install.cmake")
  include("/home/hzyu/git/gtsam/build/gtsam/navigation/cmake_install.cmake")

endif()

