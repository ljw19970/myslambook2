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
CMAKE_SOURCE_DIR = /home/lee/Resources/slam14/ch2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lee/Resources/slam14/ch2/build

# Include any dependencies generated for this target.
include CMakeFiles/gdbTest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gdbTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gdbTest.dir/flags.make

CMakeFiles/gdbTest.dir/gdbTest.cpp.o: CMakeFiles/gdbTest.dir/flags.make
CMakeFiles/gdbTest.dir/gdbTest.cpp.o: ../gdbTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lee/Resources/slam14/ch2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gdbTest.dir/gdbTest.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gdbTest.dir/gdbTest.cpp.o -c /home/lee/Resources/slam14/ch2/gdbTest.cpp

CMakeFiles/gdbTest.dir/gdbTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gdbTest.dir/gdbTest.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lee/Resources/slam14/ch2/gdbTest.cpp > CMakeFiles/gdbTest.dir/gdbTest.cpp.i

CMakeFiles/gdbTest.dir/gdbTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gdbTest.dir/gdbTest.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lee/Resources/slam14/ch2/gdbTest.cpp -o CMakeFiles/gdbTest.dir/gdbTest.cpp.s

# Object files for target gdbTest
gdbTest_OBJECTS = \
"CMakeFiles/gdbTest.dir/gdbTest.cpp.o"

# External object files for target gdbTest
gdbTest_EXTERNAL_OBJECTS =

gdbTest: CMakeFiles/gdbTest.dir/gdbTest.cpp.o
gdbTest: CMakeFiles/gdbTest.dir/build.make
gdbTest: CMakeFiles/gdbTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lee/Resources/slam14/ch2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable gdbTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gdbTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gdbTest.dir/build: gdbTest

.PHONY : CMakeFiles/gdbTest.dir/build

CMakeFiles/gdbTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gdbTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gdbTest.dir/clean

CMakeFiles/gdbTest.dir/depend:
	cd /home/lee/Resources/slam14/ch2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lee/Resources/slam14/ch2 /home/lee/Resources/slam14/ch2 /home/lee/Resources/slam14/ch2/build /home/lee/Resources/slam14/ch2/build /home/lee/Resources/slam14/ch2/build/CMakeFiles/gdbTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gdbTest.dir/depend
