# Install headers
file(GLOB optimizer_headers "*.h")
install(FILES ${optimizer_headers} DESTINATION include/vimp/optimizer)