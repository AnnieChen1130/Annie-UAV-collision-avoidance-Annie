# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:

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
CMAKE_COMMAND = /snap/cmake/870/bin/cmake

# The command to remove a file.
RM = /snap/cmake/870/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/uav_cpp/Desktop/Work_Branch/Annie-UAV-collision-avoidance-Annie

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/uav_cpp/Desktop/Work_Branch/Annie-UAV-collision-avoidance-Annie

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/snap/cmake/870/bin/cmake --regenerate-during-build -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/snap/cmake/870/bin/ccmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/uav_cpp/Desktop/Work_Branch/Annie-UAV-collision-avoidance-Annie/CMakeFiles /home/uav_cpp/Desktop/Work_Branch/Annie-UAV-collision-avoidance-Annie//CMakeFiles/progress.marks
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/uav_cpp/Desktop/Work_Branch/Annie-UAV-collision-avoidance-Annie/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named CA

# Build rule for target.
CA: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 CA
.PHONY : CA

# fast build rule for target.
CA/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CA.dir/build.make CMakeFiles/CA.dir/build
.PHONY : CA/fast

CA.o: CA.cpp.o
.PHONY : CA.o

# target to build an object file
CA.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CA.dir/build.make CMakeFiles/CA.dir/CA.cpp.o
.PHONY : CA.cpp.o

CA.i: CA.cpp.i
.PHONY : CA.i

# target to preprocess a source file
CA.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CA.dir/build.make CMakeFiles/CA.dir/CA.cpp.i
.PHONY : CA.cpp.i

CA.s: CA.cpp.s
.PHONY : CA.s

# target to generate assembly for a file
CA.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CA.dir/build.make CMakeFiles/CA.dir/CA.cpp.s
.PHONY : CA.cpp.s

autopilot_interface.o: autopilot_interface.cpp.o
.PHONY : autopilot_interface.o

# target to build an object file
autopilot_interface.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CA.dir/build.make CMakeFiles/CA.dir/autopilot_interface.cpp.o
.PHONY : autopilot_interface.cpp.o

autopilot_interface.i: autopilot_interface.cpp.i
.PHONY : autopilot_interface.i

# target to preprocess a source file
autopilot_interface.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CA.dir/build.make CMakeFiles/CA.dir/autopilot_interface.cpp.i
.PHONY : autopilot_interface.cpp.i

autopilot_interface.s: autopilot_interface.cpp.s
.PHONY : autopilot_interface.s

# target to generate assembly for a file
autopilot_interface.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CA.dir/build.make CMakeFiles/CA.dir/autopilot_interface.cpp.s
.PHONY : autopilot_interface.cpp.s

serial_port.o: serial_port.cpp.o
.PHONY : serial_port.o

# target to build an object file
serial_port.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CA.dir/build.make CMakeFiles/CA.dir/serial_port.cpp.o
.PHONY : serial_port.cpp.o

serial_port.i: serial_port.cpp.i
.PHONY : serial_port.i

# target to preprocess a source file
serial_port.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CA.dir/build.make CMakeFiles/CA.dir/serial_port.cpp.i
.PHONY : serial_port.cpp.i

serial_port.s: serial_port.cpp.s
.PHONY : serial_port.s

# target to generate assembly for a file
serial_port.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/CA.dir/build.make CMakeFiles/CA.dir/serial_port.cpp.s
.PHONY : serial_port.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... CA"
	@echo "... CA.o"
	@echo "... CA.i"
	@echo "... CA.s"
	@echo "... autopilot_interface.o"
	@echo "... autopilot_interface.i"
	@echo "... autopilot_interface.s"
	@echo "... serial_port.o"
	@echo "... serial_port.i"
	@echo "... serial_port.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

