#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vimp" for configuration "Release"
set_property(TARGET vimp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vimp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvimp.so"
  IMPORTED_SONAME_RELEASE "libvimp.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS vimp )
list(APPEND _IMPORT_CHECK_FILES_FOR_vimp "${_IMPORT_PREFIX}/lib/libvimp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
