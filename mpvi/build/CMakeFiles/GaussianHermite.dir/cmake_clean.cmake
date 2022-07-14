file(REMOVE_RECURSE
  "libGaussianHermiteDebug.pdb"
  "libGaussianHermiteDebug.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/GaussianHermite.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
