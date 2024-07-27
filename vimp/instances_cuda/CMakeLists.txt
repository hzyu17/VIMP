# Install headers
file(GLOB instances_headers "*.h")
install(FILES ${instances_headers} DESTINATION include/vimp/instances)

file(GLOB instances_headers "pgcs/*.h")
install(FILES ${instances_headers} DESTINATION include/vimp/instances/pgcs)