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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zokta/V3D/tp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zokta/V3D/build

# Include any dependencies generated for this target.
include CMakeFiles/TP1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/TP1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TP1.dir/flags.make

CMakeFiles/TP1.dir/TP1.cpp.o: CMakeFiles/TP1.dir/flags.make
CMakeFiles/TP1.dir/TP1.cpp.o: /home/zokta/V3D/tp/TP1.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/zokta/V3D/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/TP1.dir/TP1.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/TP1.dir/TP1.cpp.o -c /home/zokta/V3D/tp/TP1.cpp

CMakeFiles/TP1.dir/TP1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TP1.dir/TP1.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/zokta/V3D/tp/TP1.cpp > CMakeFiles/TP1.dir/TP1.cpp.i

CMakeFiles/TP1.dir/TP1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TP1.dir/TP1.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/zokta/V3D/tp/TP1.cpp -o CMakeFiles/TP1.dir/TP1.cpp.s

CMakeFiles/TP1.dir/TP1.cpp.o.requires:
.PHONY : CMakeFiles/TP1.dir/TP1.cpp.o.requires

CMakeFiles/TP1.dir/TP1.cpp.o.provides: CMakeFiles/TP1.dir/TP1.cpp.o.requires
	$(MAKE) -f CMakeFiles/TP1.dir/build.make CMakeFiles/TP1.dir/TP1.cpp.o.provides.build
.PHONY : CMakeFiles/TP1.dir/TP1.cpp.o.provides

CMakeFiles/TP1.dir/TP1.cpp.o.provides.build: CMakeFiles/TP1.dir/TP1.cpp.o

# Object files for target TP1
TP1_OBJECTS = \
"CMakeFiles/TP1.dir/TP1.cpp.o"

# External object files for target TP1
TP1_EXTERNAL_OBJECTS =

TP1: CMakeFiles/TP1.dir/TP1.cpp.o
TP1: /usr/local/lib/libvisp.a
TP1: /usr/lib/x86_64-linux-gnu/libX11.so
TP1: /usr/lib/x86_64-linux-gnu/libXext.so
TP1: /usr/lib/x86_64-linux-gnu/libm.so
TP1: /usr/lib/x86_64-linux-gnu/libpthread.so
TP1: CMakeFiles/TP1.dir/build.make
TP1: CMakeFiles/TP1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable TP1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TP1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TP1.dir/build: TP1
.PHONY : CMakeFiles/TP1.dir/build

CMakeFiles/TP1.dir/requires: CMakeFiles/TP1.dir/TP1.cpp.o.requires
.PHONY : CMakeFiles/TP1.dir/requires

CMakeFiles/TP1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TP1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TP1.dir/clean

CMakeFiles/TP1.dir/depend:
	cd /home/zokta/V3D/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zokta/V3D/tp /home/zokta/V3D/tp /home/zokta/V3D/build /home/zokta/V3D/build /home/zokta/V3D/build/CMakeFiles/TP1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TP1.dir/depend

