# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hzyu/git/gpmp2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hzyu/git/gpmp2/build

# Include any dependencies generated for this target.
include gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/depend.make

# Include the progress variables for this target.
include gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/progress.make

# Include the compile flags for this target's objects.
include gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/flags.make

gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.o: gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/flags.make
gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.o: ../gpmp2/obstacle/tests/testSelfCollision.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hzyu/git/gpmp2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.o"
	cd /home/hzyu/git/gpmp2/build/gpmp2/obstacle && /usr/bin/c++  $(CXX_DEFINES) -DTOPSRCDIR=\"/home/hzyu/git/gpmp2\" $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.o -c /home/hzyu/git/gpmp2/gpmp2/obstacle/tests/testSelfCollision.cpp

gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.i"
	cd /home/hzyu/git/gpmp2/build/gpmp2/obstacle && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/hzyu/git/gpmp2\" $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hzyu/git/gpmp2/gpmp2/obstacle/tests/testSelfCollision.cpp > CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.i

gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.s"
	cd /home/hzyu/git/gpmp2/build/gpmp2/obstacle && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/hzyu/git/gpmp2\" $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hzyu/git/gpmp2/gpmp2/obstacle/tests/testSelfCollision.cpp -o CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.s

# Object files for target testSelfCollision
testSelfCollision_OBJECTS = \
"CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.o"

# External object files for target testSelfCollision
testSelfCollision_EXTERNAL_OBJECTS =

gpmp2/obstacle/testSelfCollision: gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/tests/testSelfCollision.cpp.o
gpmp2/obstacle/testSelfCollision: gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/build.make
gpmp2/obstacle/testSelfCollision: /usr/local/lib/libCppUnitLite.a
gpmp2/obstacle/testSelfCollision: gpmp2/libgpmp2.so.0.3.0
gpmp2/obstacle/testSelfCollision: /usr/local/lib/libgtsam.so.4.0.0
gpmp2/obstacle/testSelfCollision: /usr/local/lib/libboost_serialization.so.1.78.0
gpmp2/obstacle/testSelfCollision: /usr/local/lib/libboost_system.so.1.78.0
gpmp2/obstacle/testSelfCollision: /usr/local/lib/libboost_filesystem.so.1.78.0
gpmp2/obstacle/testSelfCollision: /usr/local/lib/libboost_atomic.so.1.78.0
gpmp2/obstacle/testSelfCollision: /usr/local/lib/libboost_thread.so.1.78.0
gpmp2/obstacle/testSelfCollision: /usr/local/lib/libboost_date_time.so.1.78.0
gpmp2/obstacle/testSelfCollision: /usr/local/lib/libboost_timer.so.1.78.0
gpmp2/obstacle/testSelfCollision: /usr/local/lib/libboost_chrono.so.1.78.0
gpmp2/obstacle/testSelfCollision: /usr/local/lib/libmetis.so
gpmp2/obstacle/testSelfCollision: gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hzyu/git/gpmp2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable testSelfCollision"
	cd /home/hzyu/git/gpmp2/build/gpmp2/obstacle && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testSelfCollision.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/build: gpmp2/obstacle/testSelfCollision

.PHONY : gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/build

gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/clean:
	cd /home/hzyu/git/gpmp2/build/gpmp2/obstacle && $(CMAKE_COMMAND) -P CMakeFiles/testSelfCollision.dir/cmake_clean.cmake
.PHONY : gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/clean

gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/depend:
	cd /home/hzyu/git/gpmp2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hzyu/git/gpmp2 /home/hzyu/git/gpmp2/gpmp2/obstacle /home/hzyu/git/gpmp2/build /home/hzyu/git/gpmp2/build/gpmp2/obstacle /home/hzyu/git/gpmp2/build/gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gpmp2/obstacle/CMakeFiles/testSelfCollision.dir/depend

