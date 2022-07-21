file(REMOVE_RECURSE
  "libvimp.pdb"
  "libvimp.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/vimp.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
