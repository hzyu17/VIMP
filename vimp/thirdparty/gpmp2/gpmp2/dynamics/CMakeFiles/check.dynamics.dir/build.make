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

# Utility rule file for check.dynamics.

# Include the progress variables for this target.
include gpmp2/dynamics/CMakeFiles/check.dynamics.dir/progress.make

gpmp2/dynamics/CMakeFiles/check.dynamics:
	cd /home/hzyu/git/gpmp2/build/gpmp2/dynamics && /usr/bin/ctest -C Release --output-on-failure

check.dynamics: gpmp2/dynamics/CMakeFiles/check.dynamics
check.dynamics: gpmp2/dynamics/CMakeFiles/check.dynamics.dir/build.make

.PHONY : check.dynamics

# Rule to build all files generated by this target.
gpmp2/dynamics/CMakeFiles/check.dynamics.dir/build: check.dynamics

.PHONY : gpmp2/dynamics/CMakeFiles/check.dynamics.dir/build

gpmp2/dynamics/CMakeFiles/check.dynamics.dir/clean:
	cd /home/hzyu/git/gpmp2/build/gpmp2/dynamics && $(CMAKE_COMMAND) -P CMakeFiles/check.dynamics.dir/cmake_clean.cmake
.PHONY : gpmp2/dynamics/CMakeFiles/check.dynamics.dir/clean

gpmp2/dynamics/CMakeFiles/check.dynamics.dir/depend:
	cd /home/hzyu/git/gpmp2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hzyu/git/gpmp2 /home/hzyu/git/gpmp2/gpmp2/dynamics /home/hzyu/git/gpmp2/build /home/hzyu/git/gpmp2/build/gpmp2/dynamics /home/hzyu/git/gpmp2/build/gpmp2/dynamics/CMakeFiles/check.dynamics.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gpmp2/dynamics/CMakeFiles/check.dynamics.dir/depend

