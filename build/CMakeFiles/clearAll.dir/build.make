# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cm654063/TopoRecognizer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cm654063/TopoRecognizer/build

# Utility rule file for clearAll.

# Include the progress variables for this target.
include CMakeFiles/clearAll.dir/progress.make

CMakeFiles/clearAll:
	/usr/bin/cmake -E remove *.png *.gnu *.ori Level* *.txt *.gif
	/usr/bin/cmake -E echo rm\ *.png\ *.gnu\ *.ori\ *.gif

clearAll: CMakeFiles/clearAll
clearAll: CMakeFiles/clearAll.dir/build.make
.PHONY : clearAll

# Rule to build all files generated by this target.
CMakeFiles/clearAll.dir/build: clearAll
.PHONY : CMakeFiles/clearAll.dir/build

CMakeFiles/clearAll.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clearAll.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clearAll.dir/clean

CMakeFiles/clearAll.dir/depend:
	cd /home/cm654063/TopoRecognizer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cm654063/TopoRecognizer /home/cm654063/TopoRecognizer /home/cm654063/TopoRecognizer/build /home/cm654063/TopoRecognizer/build /home/cm654063/TopoRecognizer/build/CMakeFiles/clearAll.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/clearAll.dir/depend

