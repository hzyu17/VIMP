file(REMOVE_RECURSE
  "libSparseInverserDebug.pdb"
  "libSparseInverserDebug.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/SparseInverser.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
