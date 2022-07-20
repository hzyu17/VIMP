# Install headers
file(GLOB helpers_headers "*.h")
install(FILES ${helpers_headers} DESTINATION include/vimp/helpers)