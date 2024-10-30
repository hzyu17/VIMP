# Install headers
file(GLOB instances_headers "*.h")
install(FILES ${instances_headers} DESTINATION include/vimp/instances_cuda)

file(GLOB instances_headers "pgcs/*.h")
install(FILES ${instances_headers} DESTINATION include/vimp/instances_cuda/pgcs)