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
include gpmp2/kinematics/CMakeFiles/testArm.dir/depend.make

# Include the progress variables for this target.
include gpmp2/kinematics/CMakeFiles/testArm.dir/progress.make

# Include the compile flags for this target's objects.
include gpmp2/kinematics/CMakeFiles/testArm.dir/flags.make

gpmp2/kinematics/CMakeFiles/testArm.dir/tests/testArm.cpp.o: gpmp2/kinematics/CMakeFiles/testArm.dir/flags.make
gpmp2/kinematics/CMakeFiles/testArm.dir/tests/testArm.cpp.o: ../gpmp2/kinematics/tests/testArm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hzyu/git/gpmp2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gpmp2/kinematics/CMakeFiles/testArm.dir/tests/testArm.cpp.o"
	cd /home/hzyu/git/gpmp2/build/gpmp2/kinematics && /usr/bin/c++  $(CXX_DEFINES) -DTOPSRCDIR=\"/home/hzyu/git/gpmp2\" $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testArm.dir/tests/testArm.cpp.o -c /home/hzyu/git/gpmp2/gpmp2/kinematics/tests/testArm.cpp

gpmp2/kinematics/CMakeFiles/testArm.dir/tests/testArm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testArm.dir/tests/testArm.cpp.i"
	cd /home/hzyu/git/gpmp2/build/gpmp2/kinematics && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/hzyu/git/gpmp2\" $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hzyu/git/gpmp2/gpmp2/kinematics/tests/testArm.cpp > CMakeFiles/testArm.dir/tests/testArm.cpp.i

gpmp2/kinematics/CMakeFiles/testArm.dir/tests/testArm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testArm.dir/tests/testArm.cpp.s"
	cd /home/hzyu/git/gpmp2/build/gpmp2/kinematics && /usr/bin/c++ $(CXX_DEFINES) -DTOPSRCDIR=\"/home/hzyu/git/gpmp2\" $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hzyu/git/gpmp2/gpmp2/kinematics/tests/testArm.cpp -o CMakeFiles/testArm.dir/tests/testArm.cpp.s

# Object files for target testArm
testArm_OBJECTS = \
"CMakeFiles/testArm.dir/tests/testArm.cpp.o"

# External object files for target testArm
testArm_EXTERNAL_OBJECTS =

gpmp2/kinematics/testArm: gpmp2/kinematics/CMakeFiles/testArm.dir/tests/testArm.cpp.o
gpmp2/kinematics/testArm: gpmp2/kinematics/CMakeFiles/testArm.dir/build.make
gpmp2/kinematics/testArm: /usr/local/lib/libCppUnitLite.a
gpmp2/kinematics/testArm: gpmp2/libgpmp2.so.0.3.0
gpmp2/kinematics/testArm: /usr/local/lib/libgtsam.so.4.0.0
gpmp2/kinematics/testArm: /usr/local/lib/libboost_serialization.so.1.78.0
gpmp2/kinematics/testArm: /usr/local/lib/libboost_system.so.1.78.0
gpmp2/kinematics/testArm: /usr/local/lib/libboost_filesystem.so.1.78.0
gpmp2/kinematics/testArm: /usr/local/lib/libboost_atomic.so.1.78.0
gpmp2/kinematics/testArm: /usr/local/lib/libboost_thread.so.1.78.0
gpmp2/kinematics/testArm: /usr/local/lib/libboost_date_time.so.1.78.0
gpmp2/kinematics/testArm: /usr/local/lib/libboost_timer.so.1.78.0
gpmp2/kinematics/testArm: /usr/local/lib/libboost_chrono.so.1.78.0
gpmp2/kinematics/testArm: /usr/local/lib/libmetis.so
gpmp2/kinematics/testArm: gpmp2/kinematics/CMakeFiles/testArm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hzyu/git/gpmp2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable testArm"
	cd /home/hzyu/git/gpmp2/build/gpmp2/kinematics && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testArm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gpmp2/kinematics/CMakeFiles/testArm.dir/build: gpmp2/kinematics/testArm

.PHONY : gpmp2/kinematics/CMakeFiles/testArm.dir/build

gpmp2/kinematics/CMakeFiles/testArm.dir/clean:
	cd /home/hzyu/git/gpmp2/build/gpmp2/kinematics && $(CMAKE_COMMAND) -P CMakeFiles/testArm.dir/cmake_clean.cmake
.PHONY : gpmp2/kinematics/CMakeFiles/testArm.dir/clean

gpmp2/kinematics/CMakeFiles/testArm.dir/depend:
	cd /home/hzyu/git/gpmp2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hzyu/git/gpmp2 /home/hzyu/git/gpmp2/gpmp2/kinematics /home/hzyu/git/gpmp2/build /home/hzyu/git/gpmp2/build/gpmp2/kinematics /home/hzyu/git/gpmp2/build/gpmp2/kinematics/CMakeFiles/testArm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gpmp2/kinematics/CMakeFiles/testArm.dir/depend

