file(REMOVE_RECURSE
  "libSpHelperDebug.pdb"
  "libSpHelperDebug.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/SpHelper.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
