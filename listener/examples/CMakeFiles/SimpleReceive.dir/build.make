# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/dan/oscpack/tags/release_1_1_0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dan/oscpack/tags/release_1_1_0/examples

# Include any dependencies generated for this target.
include CMakeFiles/SimpleReceive.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SimpleReceive.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SimpleReceive.dir/flags.make

CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o: CMakeFiles/SimpleReceive.dir/flags.make
CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o: SimpleReceive.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dan/oscpack/tags/release_1_1_0/examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o -c /home/dan/oscpack/tags/release_1_1_0/examples/SimpleReceive.cpp

CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dan/oscpack/tags/release_1_1_0/examples/SimpleReceive.cpp > CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.i

CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dan/oscpack/tags/release_1_1_0/examples/SimpleReceive.cpp -o CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.s

CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o.requires:

.PHONY : CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o.requires

CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o.provides: CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o.requires
	$(MAKE) -f CMakeFiles/SimpleReceive.dir/build.make CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o.provides.build
.PHONY : CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o.provides

CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o.provides.build: CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o


# Object files for target SimpleReceive
SimpleReceive_OBJECTS = \
"CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o"

# External object files for target SimpleReceive
SimpleReceive_EXTERNAL_OBJECTS =

SimpleReceive: CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o
SimpleReceive: CMakeFiles/SimpleReceive.dir/build.make
SimpleReceive: liboscpack.a
SimpleReceive: CMakeFiles/SimpleReceive.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dan/oscpack/tags/release_1_1_0/examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable SimpleReceive"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SimpleReceive.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SimpleReceive.dir/build: SimpleReceive

.PHONY : CMakeFiles/SimpleReceive.dir/build

CMakeFiles/SimpleReceive.dir/requires: CMakeFiles/SimpleReceive.dir/SimpleReceive.cpp.o.requires

.PHONY : CMakeFiles/SimpleReceive.dir/requires

CMakeFiles/SimpleReceive.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SimpleReceive.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SimpleReceive.dir/clean

CMakeFiles/SimpleReceive.dir/depend:
	cd /home/dan/oscpack/tags/release_1_1_0/examples && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dan/oscpack/tags/release_1_1_0 /home/dan/oscpack/tags/release_1_1_0 /home/dan/oscpack/tags/release_1_1_0/examples /home/dan/oscpack/tags/release_1_1_0/examples /home/dan/oscpack/tags/release_1_1_0/examples/CMakeFiles/SimpleReceive.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SimpleReceive.dir/depend

