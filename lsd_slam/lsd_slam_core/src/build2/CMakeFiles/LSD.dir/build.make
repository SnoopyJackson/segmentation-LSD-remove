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
CMAKE_SOURCE_DIR = /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2

# Include any dependencies generated for this target.
include CMakeFiles/LSD.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/LSD.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/LSD.dir/flags.make

CMakeFiles/LSD.dir/LSD.cpp.o: CMakeFiles/LSD.dir/flags.make
CMakeFiles/LSD.dir/LSD.cpp.o: ../LSD.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LSD.dir/LSD.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LSD.dir/LSD.cpp.o -c /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/LSD.cpp

CMakeFiles/LSD.dir/LSD.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD.dir/LSD.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/LSD.cpp > CMakeFiles/LSD.dir/LSD.cpp.i

CMakeFiles/LSD.dir/LSD.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD.dir/LSD.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/LSD.cpp -o CMakeFiles/LSD.dir/LSD.cpp.s

CMakeFiles/LSD.dir/LSD.cpp.o.requires:
.PHONY : CMakeFiles/LSD.dir/LSD.cpp.o.requires

CMakeFiles/LSD.dir/LSD.cpp.o.provides: CMakeFiles/LSD.dir/LSD.cpp.o.requires
	$(MAKE) -f CMakeFiles/LSD.dir/build.make CMakeFiles/LSD.dir/LSD.cpp.o.provides.build
.PHONY : CMakeFiles/LSD.dir/LSD.cpp.o.provides

CMakeFiles/LSD.dir/LSD.cpp.o.provides.build: CMakeFiles/LSD.dir/LSD.cpp.o

CMakeFiles/LSD.dir/osg_gui.cpp.o: CMakeFiles/LSD.dir/flags.make
CMakeFiles/LSD.dir/osg_gui.cpp.o: ../osg_gui.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LSD.dir/osg_gui.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LSD.dir/osg_gui.cpp.o -c /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/osg_gui.cpp

CMakeFiles/LSD.dir/osg_gui.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD.dir/osg_gui.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/osg_gui.cpp > CMakeFiles/LSD.dir/osg_gui.cpp.i

CMakeFiles/LSD.dir/osg_gui.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD.dir/osg_gui.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/osg_gui.cpp -o CMakeFiles/LSD.dir/osg_gui.cpp.s

CMakeFiles/LSD.dir/osg_gui.cpp.o.requires:
.PHONY : CMakeFiles/LSD.dir/osg_gui.cpp.o.requires

CMakeFiles/LSD.dir/osg_gui.cpp.o.provides: CMakeFiles/LSD.dir/osg_gui.cpp.o.requires
	$(MAKE) -f CMakeFiles/LSD.dir/build.make CMakeFiles/LSD.dir/osg_gui.cpp.o.provides.build
.PHONY : CMakeFiles/LSD.dir/osg_gui.cpp.o.provides

CMakeFiles/LSD.dir/osg_gui.cpp.o.provides.build: CMakeFiles/LSD.dir/osg_gui.cpp.o

CMakeFiles/LSD.dir/camera.cpp.o: CMakeFiles/LSD.dir/flags.make
CMakeFiles/LSD.dir/camera.cpp.o: ../camera.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LSD.dir/camera.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LSD.dir/camera.cpp.o -c /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/camera.cpp

CMakeFiles/LSD.dir/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD.dir/camera.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/camera.cpp > CMakeFiles/LSD.dir/camera.cpp.i

CMakeFiles/LSD.dir/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD.dir/camera.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/camera.cpp -o CMakeFiles/LSD.dir/camera.cpp.s

CMakeFiles/LSD.dir/camera.cpp.o.requires:
.PHONY : CMakeFiles/LSD.dir/camera.cpp.o.requires

CMakeFiles/LSD.dir/camera.cpp.o.provides: CMakeFiles/LSD.dir/camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/LSD.dir/build.make CMakeFiles/LSD.dir/camera.cpp.o.provides.build
.PHONY : CMakeFiles/LSD.dir/camera.cpp.o.provides

CMakeFiles/LSD.dir/camera.cpp.o.provides.build: CMakeFiles/LSD.dir/camera.cpp.o

CMakeFiles/LSD.dir/Segmentation.cpp.o: CMakeFiles/LSD.dir/flags.make
CMakeFiles/LSD.dir/Segmentation.cpp.o: ../Segmentation.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LSD.dir/Segmentation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LSD.dir/Segmentation.cpp.o -c /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/Segmentation.cpp

CMakeFiles/LSD.dir/Segmentation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD.dir/Segmentation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/Segmentation.cpp > CMakeFiles/LSD.dir/Segmentation.cpp.i

CMakeFiles/LSD.dir/Segmentation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD.dir/Segmentation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/Segmentation.cpp -o CMakeFiles/LSD.dir/Segmentation.cpp.s

CMakeFiles/LSD.dir/Segmentation.cpp.o.requires:
.PHONY : CMakeFiles/LSD.dir/Segmentation.cpp.o.requires

CMakeFiles/LSD.dir/Segmentation.cpp.o.provides: CMakeFiles/LSD.dir/Segmentation.cpp.o.requires
	$(MAKE) -f CMakeFiles/LSD.dir/build.make CMakeFiles/LSD.dir/Segmentation.cpp.o.provides.build
.PHONY : CMakeFiles/LSD.dir/Segmentation.cpp.o.provides

CMakeFiles/LSD.dir/Segmentation.cpp.o.provides.build: CMakeFiles/LSD.dir/Segmentation.cpp.o

CMakeFiles/LSD.dir/MainPlane.cpp.o: CMakeFiles/LSD.dir/flags.make
CMakeFiles/LSD.dir/MainPlane.cpp.o: ../MainPlane.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LSD.dir/MainPlane.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LSD.dir/MainPlane.cpp.o -c /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/MainPlane.cpp

CMakeFiles/LSD.dir/MainPlane.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD.dir/MainPlane.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/MainPlane.cpp > CMakeFiles/LSD.dir/MainPlane.cpp.i

CMakeFiles/LSD.dir/MainPlane.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD.dir/MainPlane.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/MainPlane.cpp -o CMakeFiles/LSD.dir/MainPlane.cpp.s

CMakeFiles/LSD.dir/MainPlane.cpp.o.requires:
.PHONY : CMakeFiles/LSD.dir/MainPlane.cpp.o.requires

CMakeFiles/LSD.dir/MainPlane.cpp.o.provides: CMakeFiles/LSD.dir/MainPlane.cpp.o.requires
	$(MAKE) -f CMakeFiles/LSD.dir/build.make CMakeFiles/LSD.dir/MainPlane.cpp.o.provides.build
.PHONY : CMakeFiles/LSD.dir/MainPlane.cpp.o.provides

CMakeFiles/LSD.dir/MainPlane.cpp.o.provides.build: CMakeFiles/LSD.dir/MainPlane.cpp.o

CMakeFiles/LSD.dir/Plane.cpp.o: CMakeFiles/LSD.dir/flags.make
CMakeFiles/LSD.dir/Plane.cpp.o: ../Plane.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LSD.dir/Plane.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LSD.dir/Plane.cpp.o -c /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/Plane.cpp

CMakeFiles/LSD.dir/Plane.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD.dir/Plane.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/Plane.cpp > CMakeFiles/LSD.dir/Plane.cpp.i

CMakeFiles/LSD.dir/Plane.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD.dir/Plane.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/Plane.cpp -o CMakeFiles/LSD.dir/Plane.cpp.s

CMakeFiles/LSD.dir/Plane.cpp.o.requires:
.PHONY : CMakeFiles/LSD.dir/Plane.cpp.o.requires

CMakeFiles/LSD.dir/Plane.cpp.o.provides: CMakeFiles/LSD.dir/Plane.cpp.o.requires
	$(MAKE) -f CMakeFiles/LSD.dir/build.make CMakeFiles/LSD.dir/Plane.cpp.o.provides.build
.PHONY : CMakeFiles/LSD.dir/Plane.cpp.o.provides

CMakeFiles/LSD.dir/Plane.cpp.o.provides.build: CMakeFiles/LSD.dir/Plane.cpp.o

CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o: CMakeFiles/LSD.dir/flags.make
CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o: ../discrete_depth_distortion_model.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o -c /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/discrete_depth_distortion_model.cpp

CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/discrete_depth_distortion_model.cpp > CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.i

CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/discrete_depth_distortion_model.cpp -o CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.s

CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o.requires:
.PHONY : CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o.requires

CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o.provides: CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o.requires
	$(MAKE) -f CMakeFiles/LSD.dir/build.make CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o.provides.build
.PHONY : CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o.provides

CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o.provides.build: CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o

CMakeFiles/LSD.dir/frame_projector.cpp.o: CMakeFiles/LSD.dir/flags.make
CMakeFiles/LSD.dir/frame_projector.cpp.o: ../frame_projector.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LSD.dir/frame_projector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LSD.dir/frame_projector.cpp.o -c /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/frame_projector.cpp

CMakeFiles/LSD.dir/frame_projector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD.dir/frame_projector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/frame_projector.cpp > CMakeFiles/LSD.dir/frame_projector.cpp.i

CMakeFiles/LSD.dir/frame_projector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD.dir/frame_projector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/frame_projector.cpp -o CMakeFiles/LSD.dir/frame_projector.cpp.s

CMakeFiles/LSD.dir/frame_projector.cpp.o.requires:
.PHONY : CMakeFiles/LSD.dir/frame_projector.cpp.o.requires

CMakeFiles/LSD.dir/frame_projector.cpp.o.provides: CMakeFiles/LSD.dir/frame_projector.cpp.o.requires
	$(MAKE) -f CMakeFiles/LSD.dir/build.make CMakeFiles/LSD.dir/frame_projector.cpp.o.provides.build
.PHONY : CMakeFiles/LSD.dir/frame_projector.cpp.o.provides

CMakeFiles/LSD.dir/frame_projector.cpp.o.provides.build: CMakeFiles/LSD.dir/frame_projector.cpp.o

CMakeFiles/LSD.dir/serializable.cpp.o: CMakeFiles/LSD.dir/flags.make
CMakeFiles/LSD.dir/serializable.cpp.o: ../serializable.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/LSD.dir/serializable.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LSD.dir/serializable.cpp.o -c /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/serializable.cpp

CMakeFiles/LSD.dir/serializable.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LSD.dir/serializable.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/serializable.cpp > CMakeFiles/LSD.dir/serializable.cpp.i

CMakeFiles/LSD.dir/serializable.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LSD.dir/serializable.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/serializable.cpp -o CMakeFiles/LSD.dir/serializable.cpp.s

CMakeFiles/LSD.dir/serializable.cpp.o.requires:
.PHONY : CMakeFiles/LSD.dir/serializable.cpp.o.requires

CMakeFiles/LSD.dir/serializable.cpp.o.provides: CMakeFiles/LSD.dir/serializable.cpp.o.requires
	$(MAKE) -f CMakeFiles/LSD.dir/build.make CMakeFiles/LSD.dir/serializable.cpp.o.provides.build
.PHONY : CMakeFiles/LSD.dir/serializable.cpp.o.provides

CMakeFiles/LSD.dir/serializable.cpp.o.provides.build: CMakeFiles/LSD.dir/serializable.cpp.o

# Object files for target LSD
LSD_OBJECTS = \
"CMakeFiles/LSD.dir/LSD.cpp.o" \
"CMakeFiles/LSD.dir/osg_gui.cpp.o" \
"CMakeFiles/LSD.dir/camera.cpp.o" \
"CMakeFiles/LSD.dir/Segmentation.cpp.o" \
"CMakeFiles/LSD.dir/MainPlane.cpp.o" \
"CMakeFiles/LSD.dir/Plane.cpp.o" \
"CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o" \
"CMakeFiles/LSD.dir/frame_projector.cpp.o" \
"CMakeFiles/LSD.dir/serializable.cpp.o"

# External object files for target LSD
LSD_EXTERNAL_OBJECTS =

LSD: CMakeFiles/LSD.dir/LSD.cpp.o
LSD: CMakeFiles/LSD.dir/osg_gui.cpp.o
LSD: CMakeFiles/LSD.dir/camera.cpp.o
LSD: CMakeFiles/LSD.dir/Segmentation.cpp.o
LSD: CMakeFiles/LSD.dir/MainPlane.cpp.o
LSD: CMakeFiles/LSD.dir/Plane.cpp.o
LSD: CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o
LSD: CMakeFiles/LSD.dir/frame_projector.cpp.o
LSD: CMakeFiles/LSD.dir/serializable.cpp.o
LSD: ../lib/liblsdslam.so
LSD: /usr/lib/libboost_thread-mt.so
LSD: /usr/lib/libboost_filesystem-mt.so
LSD: /usr/lib/libboost_system-mt.so
LSD: /usr/lib/libboost_date_time-mt.so
LSD: /usr/lib/libboost_iostreams-mt.so
LSD: /usr/lib/libboost_serialization-mt.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_common.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_octree.so
LSD: /usr/lib/libOpenNI.so
LSD: /usr/lib/libOpenNI2.so
LSD: /usr/lib/libvtkCommon.so.5.8.0
LSD: /usr/lib/libvtkFiltering.so.5.8.0
LSD: /usr/lib/libvtkImaging.so.5.8.0
LSD: /usr/lib/libvtkGraphics.so.5.8.0
LSD: /usr/lib/libvtkGenericFiltering.so.5.8.0
LSD: /usr/lib/libvtkIO.so.5.8.0
LSD: /usr/lib/libvtkRendering.so.5.8.0
LSD: /usr/lib/libvtkVolumeRendering.so.5.8.0
LSD: /usr/lib/libvtkHybrid.so.5.8.0
LSD: /usr/lib/libvtkWidgets.so.5.8.0
LSD: /usr/lib/libvtkParallel.so.5.8.0
LSD: /usr/lib/libvtkInfovis.so.5.8.0
LSD: /usr/lib/libvtkGeovis.so.5.8.0
LSD: /usr/lib/libvtkViews.so.5.8.0
LSD: /usr/lib/libvtkCharts.so.5.8.0
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_io.so
LSD: /opt/ros/fuerte/lib/libflann_cpp_s.a
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_kdtree.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_search.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_sample_consensus.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_filters.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_features.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_keypoints.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_ml.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_segmentation.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_visualization.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_outofcore.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_stereo.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_cuda_segmentation.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_cuda_sample_consensus.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_cuda_features.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_containers.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_utils.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_octree.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_segmentation.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_features.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_kinfu_large_scale.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_kinfu.so
LSD: /usr/lib/libqhull.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_surface.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_registration.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_tracking.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_recognition.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_people.so
LSD: /usr/lib/libboost_thread-mt.so
LSD: /usr/lib/libboost_filesystem-mt.so
LSD: /usr/lib/libboost_system-mt.so
LSD: /usr/lib/libboost_system-mt.so
LSD: /usr/lib/libboost_filesystem-mt.so
LSD: /usr/lib/libboost_thread-mt.so
LSD: /usr/lib/libboost_date_time-mt.so
LSD: /usr/lib/libboost_iostreams-mt.so
LSD: /usr/lib/libboost_serialization-mt.so
LSD: /usr/lib/libqhull.so
LSD: /usr/lib/libOpenNI.so
LSD: /usr/lib/libOpenNI2.so
LSD: /opt/ros/fuerte/lib/libflann_cpp_s.a
LSD: /usr/lib/libvtkCommon.so.5.8.0
LSD: /usr/lib/libvtkFiltering.so.5.8.0
LSD: /usr/lib/libvtkImaging.so.5.8.0
LSD: /usr/lib/libvtkGraphics.so.5.8.0
LSD: /usr/lib/libvtkGenericFiltering.so.5.8.0
LSD: /usr/lib/libvtkIO.so.5.8.0
LSD: /usr/lib/libvtkRendering.so.5.8.0
LSD: /usr/lib/libvtkVolumeRendering.so.5.8.0
LSD: /usr/lib/libvtkHybrid.so.5.8.0
LSD: /usr/lib/libvtkWidgets.so.5.8.0
LSD: /usr/lib/libvtkParallel.so.5.8.0
LSD: /usr/lib/libvtkInfovis.so.5.8.0
LSD: /usr/lib/libvtkGeovis.so.5.8.0
LSD: /usr/lib/libvtkViews.so.5.8.0
LSD: /usr/lib/libvtkCharts.so.5.8.0
LSD: /usr/local/lib64/libosgDB.so
LSD: /usr/local/lib64/libosgUtil.so
LSD: /usr/local/lib64/libosgViewer.so
LSD: /usr/local/lib64/libosgGA.so
LSD: /usr/local/lib64/libosg.so
LSD: /usr/local/lib64/libOpenThreads.so
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_videostab.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_ts.a
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_superres.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_stitching.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_contrib.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_nonfree.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_ocl.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_gpu.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_photo.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_objdetect.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_legacy.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_video.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_ml.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_calib3d.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_features2d.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_highgui.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_imgproc.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_flann.so.2.4.9
LSD: /home/prashanthi/opencv-2.4.9/release/lib/libopencv_core.so.2.4.9
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_io.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_kdtree.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_search.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_sample_consensus.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_filters.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_features.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_keypoints.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_ml.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_segmentation.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_visualization.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_outofcore.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_stereo.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_cuda_segmentation.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_cuda_sample_consensus.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_cuda_features.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_containers.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_utils.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_octree.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_segmentation.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_features.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_kinfu_large_scale.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_gpu_kinfu.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_surface.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_registration.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_tracking.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_recognition.so
LSD: /home/prashanthi/pcl-trunk/build/lib/libpcl_people.so
LSD: /usr/lib/libboost_thread-mt.so
LSD: /usr/lib/libboost_filesystem-mt.so
LSD: /usr/lib/libboost_system-mt.so
LSD: /usr/lib/libboost_thread-mt.so
LSD: /usr/lib/libboost_filesystem-mt.so
LSD: /usr/lib/libboost_system-mt.so
LSD: /usr/lib/libboost_date_time-mt.so
LSD: /usr/lib/libboost_iostreams-mt.so
LSD: /usr/lib/libboost_serialization-mt.so
LSD: /usr/local/lib64/libosgDB.so
LSD: /usr/local/lib64/libosgUtil.so
LSD: /usr/local/lib64/libosgViewer.so
LSD: /usr/local/lib64/libosgGA.so
LSD: /usr/local/lib64/libosg.so
LSD: /usr/local/lib64/libOpenThreads.so
LSD: /usr/lib/libvtkViews.so.5.8.0
LSD: /usr/lib/libvtkInfovis.so.5.8.0
LSD: /usr/lib/libvtkWidgets.so.5.8.0
LSD: /usr/lib/libvtkVolumeRendering.so.5.8.0
LSD: /usr/lib/libvtkHybrid.so.5.8.0
LSD: /usr/lib/libvtkParallel.so.5.8.0
LSD: /usr/lib/libvtkRendering.so.5.8.0
LSD: /usr/lib/libvtkImaging.so.5.8.0
LSD: /usr/lib/libvtkGraphics.so.5.8.0
LSD: /usr/lib/libvtkIO.so.5.8.0
LSD: /usr/lib/libvtkFiltering.so.5.8.0
LSD: /usr/lib/libvtkCommon.so.5.8.0
LSD: /usr/lib/libvtksys.so.5.8.0
LSD: CMakeFiles/LSD.dir/build.make
LSD: CMakeFiles/LSD.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable LSD"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LSD.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/LSD.dir/build: LSD
.PHONY : CMakeFiles/LSD.dir/build

CMakeFiles/LSD.dir/requires: CMakeFiles/LSD.dir/LSD.cpp.o.requires
CMakeFiles/LSD.dir/requires: CMakeFiles/LSD.dir/osg_gui.cpp.o.requires
CMakeFiles/LSD.dir/requires: CMakeFiles/LSD.dir/camera.cpp.o.requires
CMakeFiles/LSD.dir/requires: CMakeFiles/LSD.dir/Segmentation.cpp.o.requires
CMakeFiles/LSD.dir/requires: CMakeFiles/LSD.dir/MainPlane.cpp.o.requires
CMakeFiles/LSD.dir/requires: CMakeFiles/LSD.dir/Plane.cpp.o.requires
CMakeFiles/LSD.dir/requires: CMakeFiles/LSD.dir/discrete_depth_distortion_model.cpp.o.requires
CMakeFiles/LSD.dir/requires: CMakeFiles/LSD.dir/frame_projector.cpp.o.requires
CMakeFiles/LSD.dir/requires: CMakeFiles/LSD.dir/serializable.cpp.o.requires
.PHONY : CMakeFiles/LSD.dir/requires

CMakeFiles/LSD.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/LSD.dir/cmake_clean.cmake
.PHONY : CMakeFiles/LSD.dir/clean

CMakeFiles/LSD.dir/depend:
	cd /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2 /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2 /home/prashanthi/projets/segmentation_thread_tracking_remove/lsd_slam/lsd_slam_core/src/build2/CMakeFiles/LSD.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/LSD.dir/depend
