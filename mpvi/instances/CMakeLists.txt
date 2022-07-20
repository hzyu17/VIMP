# Install headers
file(GLOB instances_headers "*.h")
install(FILES ${instances_headers} DESTINATION include/vimp/instances)