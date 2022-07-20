file(REMOVE_RECURSE
  "libFactorizedOptimizerDebug.pdb"
  "libFactorizedOptimizerDebug.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/FactorizedOptimizer.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
