# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/opencv4".split(';') if "${prefix}/include;/usr/include/opencv4" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;dynamic_reconfigure;message_filters;sensor_msgs;vision_msgs;visualization_msgs;cv_bridge;image_transport;nodelet".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldepthai_filters;/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0".split(';') if "-ldepthai_filters;/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0" != "" else []
PROJECT_NAME = "depthai_filters"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "2.9.0"
