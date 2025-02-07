# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

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
CMAKE_COMMAND = /snap/clion/151/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/151/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/laravelk/project_tprt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/laravelk/project_tprt

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/snap/clion/151/bin/cmake/linux/bin/cmake --regenerate-during-build -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/snap/clion/151/bin/cmake/linux/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/laravelk/project_tprt/CMakeFiles /home/laravelk/project_tprt//CMakeFiles/progress.marks
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/laravelk/project_tprt/CMakeFiles 0
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
# Target rules for targets named tprt

# Build rule for target.
tprt: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 tprt
.PHONY : tprt

# fast build rule for target.
tprt/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/build
.PHONY : tprt/fast

Data/Horizon/FlatHorizon.o: Data/Horizon/FlatHorizon.cpp.o

.PHONY : Data/Horizon/FlatHorizon.o

# target to build an object file
Data/Horizon/FlatHorizon.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Horizon/FlatHorizon.cpp.o
.PHONY : Data/Horizon/FlatHorizon.cpp.o

Data/Horizon/FlatHorizon.i: Data/Horizon/FlatHorizon.cpp.i

.PHONY : Data/Horizon/FlatHorizon.i

# target to preprocess a source file
Data/Horizon/FlatHorizon.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Horizon/FlatHorizon.cpp.i
.PHONY : Data/Horizon/FlatHorizon.cpp.i

Data/Horizon/FlatHorizon.s: Data/Horizon/FlatHorizon.cpp.s

.PHONY : Data/Horizon/FlatHorizon.s

# target to generate assembly for a file
Data/Horizon/FlatHorizon.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Horizon/FlatHorizon.cpp.s
.PHONY : Data/Horizon/FlatHorizon.cpp.s

Data/Horizon/GridHorizon.o: Data/Horizon/GridHorizon.cpp.o

.PHONY : Data/Horizon/GridHorizon.o

# target to build an object file
Data/Horizon/GridHorizon.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Horizon/GridHorizon.cpp.o
.PHONY : Data/Horizon/GridHorizon.cpp.o

Data/Horizon/GridHorizon.i: Data/Horizon/GridHorizon.cpp.i

.PHONY : Data/Horizon/GridHorizon.i

# target to preprocess a source file
Data/Horizon/GridHorizon.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Horizon/GridHorizon.cpp.i
.PHONY : Data/Horizon/GridHorizon.cpp.i

Data/Horizon/GridHorizon.s: Data/Horizon/GridHorizon.cpp.s

.PHONY : Data/Horizon/GridHorizon.s

# target to generate assembly for a file
Data/Horizon/GridHorizon.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Horizon/GridHorizon.cpp.s
.PHONY : Data/Horizon/GridHorizon.cpp.s

Data/Layer.o: Data/Layer.cpp.o

.PHONY : Data/Layer.o

# target to build an object file
Data/Layer.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Layer.cpp.o
.PHONY : Data/Layer.cpp.o

Data/Layer.i: Data/Layer.cpp.i

.PHONY : Data/Layer.i

# target to preprocess a source file
Data/Layer.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Layer.cpp.i
.PHONY : Data/Layer.cpp.i

Data/Layer.s: Data/Layer.cpp.s

.PHONY : Data/Layer.s

# target to generate assembly for a file
Data/Layer.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Layer.cpp.s
.PHONY : Data/Layer.cpp.s

Data/Receiver.o: Data/Receiver.cpp.o

.PHONY : Data/Receiver.o

# target to build an object file
Data/Receiver.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Receiver.cpp.o
.PHONY : Data/Receiver.cpp.o

Data/Receiver.i: Data/Receiver.cpp.i

.PHONY : Data/Receiver.i

# target to preprocess a source file
Data/Receiver.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Receiver.cpp.i
.PHONY : Data/Receiver.cpp.i

Data/Receiver.s: Data/Receiver.cpp.s

.PHONY : Data/Receiver.s

# target to generate assembly for a file
Data/Receiver.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Receiver.cpp.s
.PHONY : Data/Receiver.cpp.s

Data/Source.o: Data/Source.cpp.o

.PHONY : Data/Source.o

# target to build an object file
Data/Source.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Source.cpp.o
.PHONY : Data/Source.cpp.o

Data/Source.i: Data/Source.cpp.i

.PHONY : Data/Source.i

# target to preprocess a source file
Data/Source.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Source.cpp.i
.PHONY : Data/Source.cpp.i

Data/Source.s: Data/Source.cpp.s

.PHONY : Data/Source.s

# target to generate assembly for a file
Data/Source.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/Source.cpp.s
.PHONY : Data/Source.cpp.s

Data/VelocityModel.o: Data/VelocityModel.cpp.o

.PHONY : Data/VelocityModel.o

# target to build an object file
Data/VelocityModel.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/VelocityModel.cpp.o
.PHONY : Data/VelocityModel.cpp.o

Data/VelocityModel.i: Data/VelocityModel.cpp.i

.PHONY : Data/VelocityModel.i

# target to preprocess a source file
Data/VelocityModel.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/VelocityModel.cpp.i
.PHONY : Data/VelocityModel.cpp.i

Data/VelocityModel.s: Data/VelocityModel.cpp.s

.PHONY : Data/VelocityModel.s

# target to generate assembly for a file
Data/VelocityModel.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Data/VelocityModel.cpp.s
.PHONY : Data/VelocityModel.cpp.s

Ray/Ray.o: Ray/Ray.cpp.o

.PHONY : Ray/Ray.o

# target to build an object file
Ray/Ray.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Ray/Ray.cpp.o
.PHONY : Ray/Ray.cpp.o

Ray/Ray.i: Ray/Ray.cpp.i

.PHONY : Ray/Ray.i

# target to preprocess a source file
Ray/Ray.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Ray/Ray.cpp.i
.PHONY : Ray/Ray.cpp.i

Ray/Ray.s: Ray/Ray.cpp.s

.PHONY : Ray/Ray.s

# target to generate assembly for a file
Ray/Ray.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Ray/Ray.cpp.s
.PHONY : Ray/Ray.cpp.s

Test/TestCase.o: Test/TestCase.cpp.o

.PHONY : Test/TestCase.o

# target to build an object file
Test/TestCase.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Test/TestCase.cpp.o
.PHONY : Test/TestCase.cpp.o

Test/TestCase.i: Test/TestCase.cpp.i

.PHONY : Test/TestCase.i

# target to preprocess a source file
Test/TestCase.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Test/TestCase.cpp.i
.PHONY : Test/TestCase.cpp.i

Test/TestCase.s: Test/TestCase.cpp.s

.PHONY : Test/TestCase.s

# target to generate assembly for a file
Test/TestCase.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/Test/TestCase.cpp.s
.PHONY : Test/TestCase.cpp.s

main.o: main.cpp.o

.PHONY : main.o

# target to build an object file
main.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/main.cpp.o
.PHONY : main.cpp.o

main.i: main.cpp.i

.PHONY : main.i

# target to preprocess a source file
main.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/main.cpp.i
.PHONY : main.cpp.i

main.s: main.cpp.s

.PHONY : main.s

# target to generate assembly for a file
main.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/tprt.dir/build.make CMakeFiles/tprt.dir/main.cpp.s
.PHONY : main.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... tprt"
	@echo "... Data/Horizon/FlatHorizon.o"
	@echo "... Data/Horizon/FlatHorizon.i"
	@echo "... Data/Horizon/FlatHorizon.s"
	@echo "... Data/Horizon/GridHorizon.o"
	@echo "... Data/Horizon/GridHorizon.i"
	@echo "... Data/Horizon/GridHorizon.s"
	@echo "... Data/Layer.o"
	@echo "... Data/Layer.i"
	@echo "... Data/Layer.s"
	@echo "... Data/Receiver.o"
	@echo "... Data/Receiver.i"
	@echo "... Data/Receiver.s"
	@echo "... Data/Source.o"
	@echo "... Data/Source.i"
	@echo "... Data/Source.s"
	@echo "... Data/VelocityModel.o"
	@echo "... Data/VelocityModel.i"
	@echo "... Data/VelocityModel.s"
	@echo "... Ray/Ray.o"
	@echo "... Ray/Ray.i"
	@echo "... Ray/Ray.s"
	@echo "... Test/TestCase.o"
	@echo "... Test/TestCase.i"
	@echo "... Test/TestCase.s"
	@echo "... main.o"
	@echo "... main.i"
	@echo "... main.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

