;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::VelSensorSetup)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'VelSensorSetup (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::VELSENSORSETUP")
  (make-package "SEPTENTRIO_GNSS_DRIVER::VELSENSORSETUP"))

(in-package "ROS")
;;//! \htmlinclude VelSensorSetup.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::VelSensorSetup
  :super ros::object
  :slots (_header _block_header _port _lever_arm_x _lever_arm_y _lever_arm_z ))

(defmethod septentrio_gnss_driver::VelSensorSetup
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:port __port) 0)
    ((:lever_arm_x __lever_arm_x) 0.0)
    ((:lever_arm_y __lever_arm_y) 0.0)
    ((:lever_arm_z __lever_arm_z) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _port (round __port))
   (setq _lever_arm_x (float __lever_arm_x))
   (setq _lever_arm_y (float __lever_arm_y))
   (setq _lever_arm_z (float __lever_arm_z))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:block_header
   (&rest __block_header)
   (if (keywordp (car __block_header))
       (send* _block_header __block_header)
     (progn
       (if __block_header (setq _block_header (car __block_header)))
       _block_header)))
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:lever_arm_x
   (&optional __lever_arm_x)
   (if __lever_arm_x (setq _lever_arm_x __lever_arm_x)) _lever_arm_x)
  (:lever_arm_y
   (&optional __lever_arm_y)
   (if __lever_arm_y (setq _lever_arm_y __lever_arm_y)) _lever_arm_y)
  (:lever_arm_z
   (&optional __lever_arm_z)
   (if __lever_arm_z (setq _lever_arm_z __lever_arm_z)) _lever_arm_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; septentrio_gnss_driver/BlockHeader _block_header
    (send _block_header :serialization-length)
    ;; uint8 _port
    1
    ;; float32 _lever_arm_x
    4
    ;; float32 _lever_arm_y
    4
    ;; float32 _lever_arm_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; septentrio_gnss_driver/BlockHeader _block_header
       (send _block_header :serialize s)
     ;; uint8 _port
       (write-byte _port s)
     ;; float32 _lever_arm_x
       (sys::poke _lever_arm_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lever_arm_y
       (sys::poke _lever_arm_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lever_arm_z
       (sys::poke _lever_arm_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; septentrio_gnss_driver/BlockHeader _block_header
     (send _block_header :deserialize buf ptr-) (incf ptr- (send _block_header :serialization-length))
   ;; uint8 _port
     (setq _port (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _lever_arm_x
     (setq _lever_arm_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lever_arm_y
     (setq _lever_arm_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lever_arm_z
     (setq _lever_arm_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::VelSensorSetup :md5sum-) "de5f86a0756c4a0797c2dda3d121932f")
(setf (get septentrio_gnss_driver::VelSensorSetup :datatype-) "septentrio_gnss_driver/VelSensorSetup")
(setf (get septentrio_gnss_driver::VelSensorSetup :definition-)
      "# VelSensorSetup block
# Block_Number 4244

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint8   port
float32 lever_arm_x # m
float32 lever_arm_y # m
float32 lever_arm_z # m
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
MSG: septentrio_gnss_driver/BlockHeader
# Blockheader including time header for all ROS messages that publish SBF blocks

uint8     sync_1
uint8     sync_2
uint16    crc
uint16    id
uint8     revision
uint16    length

uint32    tow # ms since week start
uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     
")



(provide :septentrio_gnss_driver/VelSensorSetup "de5f86a0756c4a0797c2dda3d121932f")


