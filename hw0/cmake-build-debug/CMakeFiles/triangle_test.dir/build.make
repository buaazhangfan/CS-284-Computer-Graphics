# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/apple/Desktop/CS284/hw0-intro

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/apple/Desktop/CS284/hw0-intro/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/triangle_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/triangle_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/triangle_test.dir/flags.make

CMakeFiles/triangle_test.dir/triangle_test.cpp.o: CMakeFiles/triangle_test.dir/flags.make
CMakeFiles/triangle_test.dir/triangle_test.cpp.o: ../triangle_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/apple/Desktop/CS284/hw0-intro/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/triangle_test.dir/triangle_test.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/triangle_test.dir/triangle_test.cpp.o -c /Users/apple/Desktop/CS284/hw0-intro/triangle_test.cpp

CMakeFiles/triangle_test.dir/triangle_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/triangle_test.dir/triangle_test.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/apple/Desktop/CS284/hw0-intro/triangle_test.cpp > CMakeFiles/triangle_test.dir/triangle_test.cpp.i

CMakeFiles/triangle_test.dir/triangle_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/triangle_test.dir/triangle_test.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/apple/Desktop/CS284/hw0-intro/triangle_test.cpp -o CMakeFiles/triangle_test.dir/triangle_test.cpp.s

CMakeFiles/triangle_test.dir/triangle_test.cpp.o.requires:

.PHONY : CMakeFiles/triangle_test.dir/triangle_test.cpp.o.requires

CMakeFiles/triangle_test.dir/triangle_test.cpp.o.provides: CMakeFiles/triangle_test.dir/triangle_test.cpp.o.requires
	$(MAKE) -f CMakeFiles/triangle_test.dir/build.make CMakeFiles/triangle_test.dir/triangle_test.cpp.o.provides.build
.PHONY : CMakeFiles/triangle_test.dir/triangle_test.cpp.o.provides

CMakeFiles/triangle_test.dir/triangle_test.cpp.o.provides.build: CMakeFiles/triangle_test.dir/triangle_test.cpp.o


# Object files for target triangle_test
triangle_test_OBJECTS = \
"CMakeFiles/triangle_test.dir/triangle_test.cpp.o"

# External object files for target triangle_test
triangle_test_EXTERNAL_OBJECTS =

triangle_test: CMakeFiles/triangle_test.dir/triangle_test.cpp.o
triangle_test: CMakeFiles/triangle_test.dir/build.make
triangle_test: CGL/src/libCGL_osx.a
triangle_test: CGL/deps/glew/libglew.a
triangle_test: CGL/deps/glfw/src/libglfw3.a
triangle_test: /usr/local/lib/libfreetype.dylib
triangle_test: /usr/local/lib/libfreetype.dylib
triangle_test: CMakeFiles/triangle_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/apple/Desktop/CS284/hw0-intro/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable triangle_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/triangle_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/triangle_test.dir/build: triangle_test

.PHONY : CMakeFiles/triangle_test.dir/build

CMakeFiles/triangle_test.dir/requires: CMakeFiles/triangle_test.dir/triangle_test.cpp.o.requires

.PHONY : CMakeFiles/triangle_test.dir/requires

CMakeFiles/triangle_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/triangle_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/triangle_test.dir/clean

CMakeFiles/triangle_test.dir/depend:
	cd /Users/apple/Desktop/CS284/hw0-intro/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/apple/Desktop/CS284/hw0-intro /Users/apple/Desktop/CS284/hw0-intro /Users/apple/Desktop/CS284/hw0-intro/cmake-build-debug /Users/apple/Desktop/CS284/hw0-intro/cmake-build-debug /Users/apple/Desktop/CS284/hw0-intro/cmake-build-debug/CMakeFiles/triangle_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/triangle_test.dir/depend

