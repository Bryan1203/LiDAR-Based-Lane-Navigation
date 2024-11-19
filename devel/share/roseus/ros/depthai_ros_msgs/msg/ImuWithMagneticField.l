;; Auto-generated. Do not edit!


(when (boundp 'depthai_ros_msgs::ImuWithMagneticField)
  (if (not (find-package "DEPTHAI_ROS_MSGS"))
    (make-package "DEPTHAI_ROS_MSGS"))
  (shadow 'ImuWithMagneticField (find-package "DEPTHAI_ROS_MSGS")))
(unless (find-package "DEPTHAI_ROS_MSGS::IMUWITHMAGNETICFIELD")
  (make-package "DEPTHAI_ROS_MSGS::IMUWITHMAGNETICFIELD"))

(in-package "ROS")
;;//! \htmlinclude ImuWithMagneticField.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass depthai_ros_msgs::ImuWithMagneticField
  :super ros::object
  :slots (_header _imu _field ))

(defmethod depthai_ros_msgs::ImuWithMagneticField
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:imu __imu) (instance sensor_msgs::Imu :init))
    ((:field __field) (instance sensor_msgs::MagneticField :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _imu __imu)
   (setq _field __field)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:field
   (&rest __field)
   (if (keywordp (car __field))
       (send* _field __field)
     (progn
       (if __field (setq _field (car __field)))
       _field)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sensor_msgs/Imu _imu
    (send _imu :serialization-length)
    ;; sensor_msgs/MagneticField _field
    (send _field :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sensor_msgs/Imu _imu
       (send _imu :serialize s)
     ;; sensor_msgs/MagneticField _field
       (send _field :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sensor_msgs/Imu _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; sensor_msgs/MagneticField _field
     (send _field :deserialize buf ptr-) (incf ptr- (send _field :serialization-length))
   ;;
   self)
  )

(setf (get depthai_ros_msgs::ImuWithMagneticField :md5sum-) "7542bff2bd62a394f0408d003d2b52df")
(setf (get depthai_ros_msgs::ImuWithMagneticField :datatype-) "depthai_ros_msgs/ImuWithMagneticField")
(setf (get depthai_ros_msgs::ImuWithMagneticField :definition-)
      "std_msgs/Header header
sensor_msgs/Imu imu
sensor_msgs/MagneticField field
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: sensor_msgs/MagneticField
 # Measurement of the Magnetic Field vector at a specific location.

 # If the covariance of the measurement is known, it should be filled in
 # (if all you know is the variance of each measurement, e.g. from the datasheet,
 #just put those along the diagonal)
 # A covariance matrix of all zeros will be interpreted as \"covariance unknown\",
 # and to use the data a covariance will have to be assumed or gotten from some
 # other source


 Header header                        # timestamp is the time the
                                      # field was measured
                                      # frame_id is the location and orientation
                                      # of the field measurement

 geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
                                      # field vector in Tesla
                                      # If your sensor does not output 3 axes,
                                      # put NaNs in the components not reported.

 float64[9] magnetic_field_covariance # Row major about x, y, z axes
                                      # 0 is interpreted as variance unknown
")



(provide :depthai_ros_msgs/ImuWithMagneticField "7542bff2bd62a394f0408d003d2b52df")


