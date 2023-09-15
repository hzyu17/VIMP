# Install script for directory: /home/hzyu/git/gpmp2

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gpmp2" TYPE FILE FILES "/home/hzyu/git/gpmp2/build/gpmp2/config.h")
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxDebug" TYPE DIRECTORY FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gpmp2_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gpmp2_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxDebug/gpmp2_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxDebug" TYPE MODULE FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2_mex/gpmp2_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gpmp2_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gpmp2_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gpmp2/build/gpmp2:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolboxDebug/gpmp2_wrapper.mexa64")
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolbox" TYPE DIRECTORY FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gpmp2_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gpmp2_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolbox/gpmp2_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolbox" TYPE MODULE FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2_mex/gpmp2_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gpmp2_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gpmp2_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gpmp2/build/gpmp2:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolbox/gpmp2_wrapper.mexa64")
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxTiming" TYPE DIRECTORY FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Tt][Ii][Mm][Ii][Nn][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Tt][Ii][Mm][Ii][Nn][Gg])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gpmp2_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gpmp2_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxTiming/gpmp2_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxTiming" TYPE MODULE FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2_mex/gpmp2_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gpmp2_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gpmp2_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gpmp2/build/gpmp2:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolboxTiming/gpmp2_wrapper.mexa64")
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxProfiling" TYPE DIRECTORY FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Pp][Rr][Oo][Ff][Ii][Ll][Ii][Nn][Gg])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Pp][Rr][Oo][Ff][Ii][Ll][Ii][Nn][Gg])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gpmp2_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gpmp2_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxProfiling/gpmp2_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxProfiling" TYPE MODULE FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2_mex/gpmp2_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gpmp2_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gpmp2_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gpmp2/build/gpmp2:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolboxProfiling/gpmp2_wrapper.mexa64")
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxRelWithDebInfo" TYPE DIRECTORY FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gpmp2_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gpmp2_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxRelWithDebInfo/gpmp2_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxRelWithDebInfo" TYPE MODULE FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2_mex/gpmp2_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gpmp2_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gpmp2_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gpmp2/build/gpmp2:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolboxRelWithDebInfo/gpmp2_wrapper.mexa64")
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
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxMinSizeRel" TYPE DIRECTORY FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2/" FILES_MATCHING REGEX "/[^/]*\\.m$")
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gpmp2_wrapper.mexa64")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gpmp2_wrapper.mexa64"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/gtsam_toolboxMinSizeRel/gpmp2_wrapper.mexa64")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/gtsam_toolboxMinSizeRel" TYPE MODULE FILES "/home/hzyu/git/gpmp2/build/wrap/gpmp2_mex/gpmp2_wrapper.mexa64")
    if(EXISTS "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gpmp2_wrapper.mexa64" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gpmp2_wrapper.mexa64")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gpmp2_wrapper.mexa64"
           OLD_RPATH "/home/hzyu/git/gpmp2/build/gpmp2:/usr/local/lib:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/gtsam_toolboxMinSizeRel/gpmp2_wrapper.mexa64")
      endif()
    endif()
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/gpmp2/gpmp2Config.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib/cmake/gpmp2" TYPE FILE FILES "/home/hzyu/git/gpmp2/build/gpmp2Config.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gpmp2/gpmp2-exports.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gpmp2/gpmp2-exports.cmake"
         "/home/hzyu/git/gpmp2/build/CMakeFiles/Export/lib/cmake/gpmp2/gpmp2-exports.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gpmp2/gpmp2-exports-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/gpmp2/gpmp2-exports.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gpmp2" TYPE FILE FILES "/home/hzyu/git/gpmp2/build/CMakeFiles/Export/lib/cmake/gpmp2/gpmp2-exports.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/gpmp2" TYPE FILE FILES "/home/hzyu/git/gpmp2/build/CMakeFiles/Export/lib/cmake/gpmp2/gpmp2-exports-release.cmake")
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/hzyu/git/gpmp2/build/gpmp2/cmake_install.cmake")
  include("/home/hzyu/git/gpmp2/build/matlab/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/hzyu/git/gpmp2/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
