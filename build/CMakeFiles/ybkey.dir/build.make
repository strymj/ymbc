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
CMAKE_SOURCE_DIR = /home/yamaji-s/researches/workspace/ymbc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yamaji-s/researches/workspace/ymbc/build

# Include any dependencies generated for this target.
include CMakeFiles/ybkey.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ybkey.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ybkey.dir/flags.make

CMakeFiles/ybkey.dir/src/ybkey.cpp.o: CMakeFiles/ybkey.dir/flags.make
CMakeFiles/ybkey.dir/src/ybkey.cpp.o: ../src/ybkey.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yamaji-s/researches/workspace/ymbc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ybkey.dir/src/ybkey.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ybkey.dir/src/ybkey.cpp.o -c /home/yamaji-s/researches/workspace/ymbc/src/ybkey.cpp

CMakeFiles/ybkey.dir/src/ybkey.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ybkey.dir/src/ybkey.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yamaji-s/researches/workspace/ymbc/src/ybkey.cpp > CMakeFiles/ybkey.dir/src/ybkey.cpp.i

CMakeFiles/ybkey.dir/src/ybkey.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ybkey.dir/src/ybkey.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yamaji-s/researches/workspace/ymbc/src/ybkey.cpp -o CMakeFiles/ybkey.dir/src/ybkey.cpp.s

CMakeFiles/ybkey.dir/src/ybkey.cpp.o.requires:

.PHONY : CMakeFiles/ybkey.dir/src/ybkey.cpp.o.requires

CMakeFiles/ybkey.dir/src/ybkey.cpp.o.provides: CMakeFiles/ybkey.dir/src/ybkey.cpp.o.requires
	$(MAKE) -f CMakeFiles/ybkey.dir/build.make CMakeFiles/ybkey.dir/src/ybkey.cpp.o.provides.build
.PHONY : CMakeFiles/ybkey.dir/src/ybkey.cpp.o.provides

CMakeFiles/ybkey.dir/src/ybkey.cpp.o.provides.build: CMakeFiles/ybkey.dir/src/ybkey.cpp.o


# Object files for target ybkey
ybkey_OBJECTS = \
"CMakeFiles/ybkey.dir/src/ybkey.cpp.o"

# External object files for target ybkey
ybkey_EXTERNAL_OBJECTS =

../bin/ybkey: CMakeFiles/ybkey.dir/src/ybkey.cpp.o
../bin/ybkey: CMakeFiles/ybkey.dir/build.make
../bin/ybkey: ../lib/libymbc.a
../bin/ybkey: CMakeFiles/ybkey.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yamaji-s/researches/workspace/ymbc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/ybkey"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ybkey.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ybkey.dir/build: ../bin/ybkey

.PHONY : CMakeFiles/ybkey.dir/build

CMakeFiles/ybkey.dir/requires: CMakeFiles/ybkey.dir/src/ybkey.cpp.o.requires

.PHONY : CMakeFiles/ybkey.dir/requires

CMakeFiles/ybkey.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ybkey.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ybkey.dir/clean

CMakeFiles/ybkey.dir/depend:
	cd /home/yamaji-s/researches/workspace/ymbc/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yamaji-s/researches/workspace/ymbc /home/yamaji-s/researches/workspace/ymbc /home/yamaji-s/researches/workspace/ymbc/build /home/yamaji-s/researches/workspace/ymbc/build /home/yamaji-s/researches/workspace/ymbc/build/CMakeFiles/ybkey.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ybkey.dir/depend

