# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /Applications/CMake.app/Contents/bin/cmake

# The command to remove a file.
RM = /Applications/CMake.app/Contents/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/asherhuang/Downloads/PhysicalEngine-Final/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/asherhuang/Downloads/PhysicalEngine-Final/build

# Include any dependencies generated for this target.
include math/CMakeFiles/math.dir/depend.make

# Include the progress variables for this target.
include math/CMakeFiles/math.dir/progress.make

# Include the compile flags for this target's objects.
include math/CMakeFiles/math.dir/flags.make

math/CMakeFiles/math.dir/camera.cpp.o: math/CMakeFiles/math.dir/flags.make
math/CMakeFiles/math.dir/camera.cpp.o: /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/asherhuang/Downloads/PhysicalEngine-Final/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object math/CMakeFiles/math.dir/camera.cpp.o"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/math.dir/camera.cpp.o -c /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/camera.cpp

math/CMakeFiles/math.dir/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/math.dir/camera.cpp.i"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/camera.cpp > CMakeFiles/math.dir/camera.cpp.i

math/CMakeFiles/math.dir/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/math.dir/camera.cpp.s"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/camera.cpp -o CMakeFiles/math.dir/camera.cpp.s

math/CMakeFiles/math.dir/color.cpp.o: math/CMakeFiles/math.dir/flags.make
math/CMakeFiles/math.dir/color.cpp.o: /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/color.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/asherhuang/Downloads/PhysicalEngine-Final/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object math/CMakeFiles/math.dir/color.cpp.o"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/math.dir/color.cpp.o -c /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/color.cpp

math/CMakeFiles/math.dir/color.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/math.dir/color.cpp.i"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/color.cpp > CMakeFiles/math.dir/color.cpp.i

math/CMakeFiles/math.dir/color.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/math.dir/color.cpp.s"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/color.cpp -o CMakeFiles/math.dir/color.cpp.s

math/CMakeFiles/math.dir/math.cpp.o: math/CMakeFiles/math.dir/flags.make
math/CMakeFiles/math.dir/math.cpp.o: /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/math.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/asherhuang/Downloads/PhysicalEngine-Final/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object math/CMakeFiles/math.dir/math.cpp.o"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/math.dir/math.cpp.o -c /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/math.cpp

math/CMakeFiles/math.dir/math.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/math.dir/math.cpp.i"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/math.cpp > CMakeFiles/math.dir/math.cpp.i

math/CMakeFiles/math.dir/math.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/math.dir/math.cpp.s"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/math.cpp -o CMakeFiles/math.dir/math.cpp.s

math/CMakeFiles/math.dir/matrix.cpp.o: math/CMakeFiles/math.dir/flags.make
math/CMakeFiles/math.dir/matrix.cpp.o: /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/matrix.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/asherhuang/Downloads/PhysicalEngine-Final/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object math/CMakeFiles/math.dir/matrix.cpp.o"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/math.dir/matrix.cpp.o -c /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/matrix.cpp

math/CMakeFiles/math.dir/matrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/math.dir/matrix.cpp.i"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/matrix.cpp > CMakeFiles/math.dir/matrix.cpp.i

math/CMakeFiles/math.dir/matrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/math.dir/matrix.cpp.s"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/matrix.cpp -o CMakeFiles/math.dir/matrix.cpp.s

math/CMakeFiles/math.dir/quaternion.cpp.o: math/CMakeFiles/math.dir/flags.make
math/CMakeFiles/math.dir/quaternion.cpp.o: /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/quaternion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/asherhuang/Downloads/PhysicalEngine-Final/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object math/CMakeFiles/math.dir/quaternion.cpp.o"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/math.dir/quaternion.cpp.o -c /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/quaternion.cpp

math/CMakeFiles/math.dir/quaternion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/math.dir/quaternion.cpp.i"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/quaternion.cpp > CMakeFiles/math.dir/quaternion.cpp.i

math/CMakeFiles/math.dir/quaternion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/math.dir/quaternion.cpp.s"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/quaternion.cpp -o CMakeFiles/math.dir/quaternion.cpp.s

math/CMakeFiles/math.dir/vector.cpp.o: math/CMakeFiles/math.dir/flags.make
math/CMakeFiles/math.dir/vector.cpp.o: /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/vector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/asherhuang/Downloads/PhysicalEngine-Final/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object math/CMakeFiles/math.dir/vector.cpp.o"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/math.dir/vector.cpp.o -c /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/vector.cpp

math/CMakeFiles/math.dir/vector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/math.dir/vector.cpp.i"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/vector.cpp > CMakeFiles/math.dir/vector.cpp.i

math/CMakeFiles/math.dir/vector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/math.dir/vector.cpp.s"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math/vector.cpp -o CMakeFiles/math.dir/vector.cpp.s

# Object files for target math
math_OBJECTS = \
"CMakeFiles/math.dir/camera.cpp.o" \
"CMakeFiles/math.dir/color.cpp.o" \
"CMakeFiles/math.dir/math.cpp.o" \
"CMakeFiles/math.dir/matrix.cpp.o" \
"CMakeFiles/math.dir/quaternion.cpp.o" \
"CMakeFiles/math.dir/vector.cpp.o"

# External object files for target math
math_EXTERNAL_OBJECTS =

math/libmath.a: math/CMakeFiles/math.dir/camera.cpp.o
math/libmath.a: math/CMakeFiles/math.dir/color.cpp.o
math/libmath.a: math/CMakeFiles/math.dir/math.cpp.o
math/libmath.a: math/CMakeFiles/math.dir/matrix.cpp.o
math/libmath.a: math/CMakeFiles/math.dir/quaternion.cpp.o
math/libmath.a: math/CMakeFiles/math.dir/vector.cpp.o
math/libmath.a: math/CMakeFiles/math.dir/build.make
math/libmath.a: math/CMakeFiles/math.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/asherhuang/Downloads/PhysicalEngine-Final/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX static library libmath.a"
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && $(CMAKE_COMMAND) -P CMakeFiles/math.dir/cmake_clean_target.cmake
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/math.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
math/CMakeFiles/math.dir/build: math/libmath.a

.PHONY : math/CMakeFiles/math.dir/build

math/CMakeFiles/math.dir/clean:
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math && $(CMAKE_COMMAND) -P CMakeFiles/math.dir/cmake_clean.cmake
.PHONY : math/CMakeFiles/math.dir/clean

math/CMakeFiles/math.dir/depend:
	cd /Users/asherhuang/Downloads/PhysicalEngine-Final/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/asherhuang/Downloads/PhysicalEngine-Final/src /Users/asherhuang/Downloads/PhysicalEngine-Final/src/math /Users/asherhuang/Downloads/PhysicalEngine-Final/build /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math /Users/asherhuang/Downloads/PhysicalEngine-Final/build/math/CMakeFiles/math.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : math/CMakeFiles/math.dir/depend

