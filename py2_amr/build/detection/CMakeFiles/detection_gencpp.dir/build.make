# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/maytus/internship_main/py2_amr/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maytus/internship_main/py2_amr/build

# Utility rule file for detection_gencpp.

# Include the progress variables for this target.
include detection/CMakeFiles/detection_gencpp.dir/progress.make

detection_gencpp: detection/CMakeFiles/detection_gencpp.dir/build.make

.PHONY : detection_gencpp

# Rule to build all files generated by this target.
detection/CMakeFiles/detection_gencpp.dir/build: detection_gencpp

.PHONY : detection/CMakeFiles/detection_gencpp.dir/build

detection/CMakeFiles/detection_gencpp.dir/clean:
	cd /home/maytus/internship_main/py2_amr/build/detection && $(CMAKE_COMMAND) -P CMakeFiles/detection_gencpp.dir/cmake_clean.cmake
.PHONY : detection/CMakeFiles/detection_gencpp.dir/clean

detection/CMakeFiles/detection_gencpp.dir/depend:
	cd /home/maytus/internship_main/py2_amr/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maytus/internship_main/py2_amr/src /home/maytus/internship_main/py2_amr/src/detection /home/maytus/internship_main/py2_amr/build /home/maytus/internship_main/py2_amr/build/detection /home/maytus/internship_main/py2_amr/build/detection/CMakeFiles/detection_gencpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : detection/CMakeFiles/detection_gencpp.dir/depend

