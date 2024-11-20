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
CMAKE_SOURCE_DIR = /home/gem/bobabee_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gem/bobabee_ws/build

# Include any dependencies generated for this target.
include hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/depend.make

# Include the progress variables for this target.
include hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/progress.make

# Include the compile flags for this target's objects.
include hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/flags.make

hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.o: hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/flags.make
hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.o: /home/gem/bobabee_ws/src/hardware_drivers/depthai-ros/depthai_examples/src/feature_tracker_publisher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gem/bobabee_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.o"
	cd /home/gem/bobabee_ws/build/hardware_drivers/depthai-ros/depthai_examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.o -c /home/gem/bobabee_ws/src/hardware_drivers/depthai-ros/depthai_examples/src/feature_tracker_publisher.cpp

hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.i"
	cd /home/gem/bobabee_ws/build/hardware_drivers/depthai-ros/depthai_examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gem/bobabee_ws/src/hardware_drivers/depthai-ros/depthai_examples/src/feature_tracker_publisher.cpp > CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.i

hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.s"
	cd /home/gem/bobabee_ws/build/hardware_drivers/depthai-ros/depthai_examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gem/bobabee_ws/src/hardware_drivers/depthai-ros/depthai_examples/src/feature_tracker_publisher.cpp -o CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.s

# Object files for target feature_tracker
feature_tracker_OBJECTS = \
"CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.o"

# External object files for target feature_tracker
feature_tracker_EXTERNAL_OBJECTS =

/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/src/feature_tracker_publisher.cpp.o
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/build.make
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /home/gem/bobabee_ws/devel/lib/libdepthai_bridge.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libcamera_info_manager.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libcamera_calibration_parsers.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libimage_transport.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libcv_bridge.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/librobot_state_publisher_solver.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libjoint_state_listener.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libkdl_parser.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/liburdf.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/librosconsole_bridge.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/liborocos-kdl.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/liborocos-kdl.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libtf2_ros.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libactionlib.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libtf2.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libmessage_filters.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libnodeletlib.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libbondcpp.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libclass_loader.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libdl.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libroslib.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/librospack.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libroscpp.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/librosconsole.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/librostime.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /opt/ros/noetic/lib/libcpp_common.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/local/lib/libdepthai-core.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/local/lib/libdepthai-core.so
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker: hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gem/bobabee_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker"
	cd /home/gem/bobabee_ws/build/hardware_drivers/depthai-ros/depthai_examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/feature_tracker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/build: /home/gem/bobabee_ws/devel/lib/depthai_examples/feature_tracker

.PHONY : hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/build

hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/clean:
	cd /home/gem/bobabee_ws/build/hardware_drivers/depthai-ros/depthai_examples && $(CMAKE_COMMAND) -P CMakeFiles/feature_tracker.dir/cmake_clean.cmake
.PHONY : hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/clean

hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/depend:
	cd /home/gem/bobabee_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gem/bobabee_ws/src /home/gem/bobabee_ws/src/hardware_drivers/depthai-ros/depthai_examples /home/gem/bobabee_ws/build /home/gem/bobabee_ws/build/hardware_drivers/depthai-ros/depthai_examples /home/gem/bobabee_ws/build/hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : hardware_drivers/depthai-ros/depthai_examples/CMakeFiles/feature_tracker.dir/depend

