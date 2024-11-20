;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::IMUSetup)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'IMUSetup (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::IMUSETUP")
  (make-package "SEPTENTRIO_GNSS_DRIVER::IMUSETUP"))

(in-package "ROS")
;;//! \htmlinclude IMUSetup.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::IMUSetup
  :super ros::object
  :slots (_header _block_header _serial_port _ant_lever_arm_x _ant_lever_arm_y _ant_lever_arm_z _theta_x _theta_y _theta_z ))

(defmethod septentrio_gnss_driver::IMUSetup
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:serial_port __serial_port) 0)
    ((:ant_lever_arm_x __ant_lever_arm_x) 0.0)
    ((:ant_lever_arm_y __ant_lever_arm_y) 0.0)
    ((:ant_lever_arm_z __ant_lever_arm_z) 0.0)
    ((:theta_x __theta_x) 0.0)
    ((:theta_y __theta_y) 0.0)
    ((:theta_z __theta_z) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _serial_port (round __serial_port))
   (setq _ant_lever_arm_x (float __ant_lever_arm_x))
   (setq _ant_lever_arm_y (float __ant_lever_arm_y))
   (setq _ant_lever_arm_z (float __ant_lever_arm_z))
   (setq _theta_x (float __theta_x))
   (setq _theta_y (float __theta_y))
   (setq _theta_z (float __theta_z))
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
  (:serial_port
   (&optional __serial_port)
   (if __serial_port (setq _serial_port __serial_port)) _serial_port)
  (:ant_lever_arm_x
   (&optional __ant_lever_arm_x)
   (if __ant_lever_arm_x (setq _ant_lever_arm_x __ant_lever_arm_x)) _ant_lever_arm_x)
  (:ant_lever_arm_y
   (&optional __ant_lever_arm_y)
   (if __ant_lever_arm_y (setq _ant_lever_arm_y __ant_lever_arm_y)) _ant_lever_arm_y)
  (:ant_lever_arm_z
   (&optional __ant_lever_arm_z)
   (if __ant_lever_arm_z (setq _ant_lever_arm_z __ant_lever_arm_z)) _ant_lever_arm_z)
  (:theta_x
   (&optional __theta_x)
   (if __theta_x (setq _theta_x __theta_x)) _theta_x)
  (:theta_y
   (&optional __theta_y)
   (if __theta_y (setq _theta_y __theta_y)) _theta_y)
  (:theta_z
   (&optional __theta_z)
   (if __theta_z (setq _theta_z __theta_z)) _theta_z)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; septentrio_gnss_driver/BlockHeader _block_header
    (send _block_header :serialization-length)
    ;; uint8 _serial_port
    1
    ;; float32 _ant_lever_arm_x
    4
    ;; float32 _ant_lever_arm_y
    4
    ;; float32 _ant_lever_arm_z
    4
    ;; float32 _theta_x
    4
    ;; float32 _theta_y
    4
    ;; float32 _theta_z
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
     ;; uint8 _serial_port
       (write-byte _serial_port s)
     ;; float32 _ant_lever_arm_x
       (sys::poke _ant_lever_arm_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ant_lever_arm_y
       (sys::poke _ant_lever_arm_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ant_lever_arm_z
       (sys::poke _ant_lever_arm_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_x
       (sys::poke _theta_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_y
       (sys::poke _theta_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_z
       (sys::poke _theta_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; septentrio_gnss_driver/BlockHeader _block_header
     (send _block_header :deserialize buf ptr-) (incf ptr- (send _block_header :serialization-length))
   ;; uint8 _serial_port
     (setq _serial_port (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _ant_lever_arm_x
     (setq _ant_lever_arm_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ant_lever_arm_y
     (setq _ant_lever_arm_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ant_lever_arm_z
     (setq _ant_lever_arm_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_x
     (setq _theta_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_y
     (setq _theta_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_z
     (setq _theta_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::IMUSetup :md5sum-) "da16656e2ba26ce4129ef1261f4a5033")
(setf (get septentrio_gnss_driver::IMUSetup :datatype-) "septentrio_gnss_driver/IMUSetup")
(setf (get septentrio_gnss_driver::IMUSetup :definition-)
      "# IMUSetup block
# Block_Number 4224

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header 

uint8   serial_port
float32 ant_lever_arm_x # m
float32 ant_lever_arm_y # m
float32 ant_lever_arm_z # m
float32 theta_x # deg
float32 theta_y # deg
float32 theta_z # deg
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



(provide :septentrio_gnss_driver/IMUSetup "da16656e2ba26ce4129ef1261f4a5033")


