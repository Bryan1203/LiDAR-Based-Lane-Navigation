# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "septentrio_gnss_driver: 26 messages, 0 services")

set(MSG_I_FLAGS "-Iseptentrio_gnss_driver:/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Idiagnostic_msgs:/opt/ros/noetic/share/diagnostic_msgs/cmake/../msg;-Igps_common:/opt/ros/noetic/share/gps_common/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(septentrio_gnss_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header:septentrio_gnss_driver/VectorInfoCart"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header:septentrio_gnss_driver/VectorInfoGeod"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg" ""
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg" ""
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header:septentrio_gnss_driver/RFBand"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg" "septentrio_gnss_driver/BlockHeader:septentrio_gnss_driver/MeasEpochChannelType1:std_msgs/Header:septentrio_gnss_driver/MeasEpochChannelType2"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg" "septentrio_gnss_driver/MeasEpochChannelType2"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg" ""
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg" ""
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg" ""
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg" NAME_WE)
add_custom_target(_septentrio_gnss_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "septentrio_gnss_driver" "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg" "septentrio_gnss_driver/BlockHeader:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_cpp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
)

### Generating Services

### Generating Module File
_generate_module_cpp(septentrio_gnss_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(septentrio_gnss_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(septentrio_gnss_driver_generate_messages septentrio_gnss_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_cpp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(septentrio_gnss_driver_gencpp)
add_dependencies(septentrio_gnss_driver_gencpp septentrio_gnss_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS septentrio_gnss_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_eus(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
)

### Generating Services

### Generating Module File
_generate_module_eus(septentrio_gnss_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(septentrio_gnss_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(septentrio_gnss_driver_generate_messages septentrio_gnss_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_eus _septentrio_gnss_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(septentrio_gnss_driver_geneus)
add_dependencies(septentrio_gnss_driver_geneus septentrio_gnss_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS septentrio_gnss_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_lisp(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
)

### Generating Services

### Generating Module File
_generate_module_lisp(septentrio_gnss_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(septentrio_gnss_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(septentrio_gnss_driver_generate_messages septentrio_gnss_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_lisp _septentrio_gnss_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(septentrio_gnss_driver_genlisp)
add_dependencies(septentrio_gnss_driver_genlisp septentrio_gnss_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS septentrio_gnss_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_nodejs(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(septentrio_gnss_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(septentrio_gnss_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(septentrio_gnss_driver_generate_messages septentrio_gnss_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_nodejs _septentrio_gnss_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(septentrio_gnss_driver_gennodejs)
add_dependencies(septentrio_gnss_driver_gennodejs septentrio_gnss_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS septentrio_gnss_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)
_generate_msg_py(septentrio_gnss_driver
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg"
  "${MSG_I_FLAGS}"
  "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
)

### Generating Services

### Generating Module File
_generate_module_py(septentrio_gnss_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(septentrio_gnss_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(septentrio_gnss_driver_generate_messages septentrio_gnss_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AIMPlusStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BaseVectorGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/BlockHeader.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/GALAuthStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFBand.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/RFStatus.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpoch.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType1.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/MeasEpochChannelType2.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PVTGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/PosCovGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ReceiverTime.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovCartesian.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelCovGeodetic.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttEuler.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/AttCovEuler.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/INSNavGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/IMUSetup.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoCart.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VectorInfoGeod.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/VelSensorSetup.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/gem/demo_ws/src/hardware_drivers/septentrio_gnss_driver/msg/ExtSensorMeas.msg" NAME_WE)
add_dependencies(septentrio_gnss_driver_generate_messages_py _septentrio_gnss_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(septentrio_gnss_driver_genpy)
add_dependencies(septentrio_gnss_driver_genpy septentrio_gnss_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS septentrio_gnss_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/septentrio_gnss_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(septentrio_gnss_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(septentrio_gnss_driver_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET diagnostic_msgs_generate_messages_cpp)
  add_dependencies(septentrio_gnss_driver_generate_messages_cpp diagnostic_msgs_generate_messages_cpp)
endif()
if(TARGET gps_common_generate_messages_cpp)
  add_dependencies(septentrio_gnss_driver_generate_messages_cpp gps_common_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/septentrio_gnss_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(septentrio_gnss_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(septentrio_gnss_driver_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET diagnostic_msgs_generate_messages_eus)
  add_dependencies(septentrio_gnss_driver_generate_messages_eus diagnostic_msgs_generate_messages_eus)
endif()
if(TARGET gps_common_generate_messages_eus)
  add_dependencies(septentrio_gnss_driver_generate_messages_eus gps_common_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/septentrio_gnss_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(septentrio_gnss_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(septentrio_gnss_driver_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET diagnostic_msgs_generate_messages_lisp)
  add_dependencies(septentrio_gnss_driver_generate_messages_lisp diagnostic_msgs_generate_messages_lisp)
endif()
if(TARGET gps_common_generate_messages_lisp)
  add_dependencies(septentrio_gnss_driver_generate_messages_lisp gps_common_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/septentrio_gnss_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(septentrio_gnss_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(septentrio_gnss_driver_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET diagnostic_msgs_generate_messages_nodejs)
  add_dependencies(septentrio_gnss_driver_generate_messages_nodejs diagnostic_msgs_generate_messages_nodejs)
endif()
if(TARGET gps_common_generate_messages_nodejs)
  add_dependencies(septentrio_gnss_driver_generate_messages_nodejs gps_common_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/septentrio_gnss_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(septentrio_gnss_driver_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(septentrio_gnss_driver_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET diagnostic_msgs_generate_messages_py)
  add_dependencies(septentrio_gnss_driver_generate_messages_py diagnostic_msgs_generate_messages_py)
endif()
if(TARGET gps_common_generate_messages_py)
  add_dependencies(septentrio_gnss_driver_generate_messages_py gps_common_generate_messages_py)
endif()
