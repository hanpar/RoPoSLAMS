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
CMAKE_SOURCE_DIR = /mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization

# Include any dependencies generated for this target.
include CMakeFiles/plot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/plot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/plot.dir/flags.make

CMakeFiles/plot.dir/plot.cpp.o: CMakeFiles/plot.dir/flags.make
CMakeFiles/plot.dir/plot.cpp.o: plot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/plot.dir/plot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/plot.dir/plot.cpp.o -c /mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization/plot.cpp

CMakeFiles/plot.dir/plot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/plot.dir/plot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization/plot.cpp > CMakeFiles/plot.dir/plot.cpp.i

CMakeFiles/plot.dir/plot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/plot.dir/plot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization/plot.cpp -o CMakeFiles/plot.dir/plot.cpp.s

# Object files for target plot
plot_OBJECTS = \
"CMakeFiles/plot.dir/plot.cpp.o"

# External object files for target plot
plot_EXTERNAL_OBJECTS =

plot: CMakeFiles/plot.dir/plot.cpp.o
plot: CMakeFiles/plot.dir/build.make
plot: /usr/local/lib/libgtsam.so.4.2.0
plot: /usr/lib/x86_64-linux-gnu/libpython3.8.so
plot: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0
plot: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
plot: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
plot: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
plot: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
plot: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
plot: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
plot: /usr/lib/x86_64-linux-gnu/libboost_timer.so.1.71.0
plot: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
plot: /usr/local/lib/libmetis-gtsam.so
plot: CMakeFiles/plot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable plot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/plot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/plot.dir/build: plot

.PHONY : CMakeFiles/plot.dir/build

CMakeFiles/plot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/plot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/plot.dir/clean

CMakeFiles/plot.dir/depend:
	cd /mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization /mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization /mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization /mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization /mnt/c/Users/adam8/Desktop/Umich/Course/ROB530/Project/eecs568-group17-project/pose_optimization/CMakeFiles/plot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/plot.dir/depend

