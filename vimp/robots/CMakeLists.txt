# Install headers
file(GLOB robot_headers "*.h")
install(FILES ${robot_headers} DESTINATION include/vimp/robots)