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
CMAKE_SOURCE_DIR = /home/hzyu/git/gtsam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hzyu/git/gtsam/build

# Utility rule file for testGlobalFunction.run.

# Include the progress variables for this target.
include wrap/tests/CMakeFiles/testGlobalFunction.run.dir/progress.make

wrap/tests/CMakeFiles/testGlobalFunction.run: wrap/tests/testGlobalFunction
	cd /home/hzyu/git/gtsam/build/wrap/tests && ./testGlobalFunction

testGlobalFunction.run: wrap/tests/CMakeFiles/testGlobalFunction.run
testGlobalFunction.run: wrap/tests/CMakeFiles/testGlobalFunction.run.dir/build.make

.PHONY : testGlobalFunction.run

# Rule to build all files generated by this target.
wrap/tests/CMakeFiles/testGlobalFunction.run.dir/build: testGlobalFunction.run

.PHONY : wrap/tests/CMakeFiles/testGlobalFunction.run.dir/build

wrap/tests/CMakeFiles/testGlobalFunction.run.dir/clean:
	cd /home/hzyu/git/gtsam/build/wrap/tests && $(CMAKE_COMMAND) -P CMakeFiles/testGlobalFunction.run.dir/cmake_clean.cmake
.PHONY : wrap/tests/CMakeFiles/testGlobalFunction.run.dir/clean

wrap/tests/CMakeFiles/testGlobalFunction.run.dir/depend:
	cd /home/hzyu/git/gtsam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hzyu/git/gtsam /home/hzyu/git/gtsam/wrap/tests /home/hzyu/git/gtsam/build /home/hzyu/git/gtsam/build/wrap/tests /home/hzyu/git/gtsam/build/wrap/tests/CMakeFiles/testGlobalFunction.run.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : wrap/tests/CMakeFiles/testGlobalFunction.run.dir/depend

