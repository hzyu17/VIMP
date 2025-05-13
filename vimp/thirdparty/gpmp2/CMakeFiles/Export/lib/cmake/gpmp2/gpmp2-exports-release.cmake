#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gpmp2" for configuration "Release"
set_property(TARGET gpmp2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(gpmp2 PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libgpmp2.so.0.3.0"
  IMPORTED_SONAME_RELEASE "libgpmp2.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS gpmp2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_gpmp2 "${_IMPORT_PREFIX}/lib/libgpmp2.so.0.3.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
