file(REMOVE_RECURSE
  "libmatplotlibDebug.pdb"
  "libmatplotlibDebug.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/matplotlib.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
