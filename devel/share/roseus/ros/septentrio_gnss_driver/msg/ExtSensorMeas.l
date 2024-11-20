;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::ExtSensorMeas)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'ExtSensorMeas (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::EXTSENSORMEAS")
  (make-package "SEPTENTRIO_GNSS_DRIVER::EXTSENSORMEAS"))

(in-package "ROS")
;;//! \htmlinclude ExtSensorMeas.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::ExtSensorMeas
  :super ros::object
  :slots (_header _block_header _n _sb_length _source _sensor_model _type _obs_info _acceleration_x _acceleration_y _acceleration_z _angular_rate_x _angular_rate_y _angular_rate_z _velocity_x _velocity_y _velocity_z _std_dev_x _std_dev_y _std_dev_z _sensor_temperature _zero_velocity_flag ))

(defmethod septentrio_gnss_driver::ExtSensorMeas
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:n __n) 0)
    ((:sb_length __sb_length) 0)
    ((:source __source) (make-array 0 :initial-element 0 :element-type :char))
    ((:sensor_model __sensor_model) (make-array 0 :initial-element 0 :element-type :char))
    ((:type __type) (make-array 0 :initial-element 0 :element-type :char))
    ((:obs_info __obs_info) (make-array 0 :initial-element 0 :element-type :char))
    ((:acceleration_x __acceleration_x) 0.0)
    ((:acceleration_y __acceleration_y) 0.0)
    ((:acceleration_z __acceleration_z) 0.0)
    ((:angular_rate_x __angular_rate_x) 0.0)
    ((:angular_rate_y __angular_rate_y) 0.0)
    ((:angular_rate_z __angular_rate_z) 0.0)
    ((:velocity_x __velocity_x) 0.0)
    ((:velocity_y __velocity_y) 0.0)
    ((:velocity_z __velocity_z) 0.0)
    ((:std_dev_x __std_dev_x) 0.0)
    ((:std_dev_y __std_dev_y) 0.0)
    ((:std_dev_z __std_dev_z) 0.0)
    ((:sensor_temperature __sensor_temperature) 0.0)
    ((:zero_velocity_flag __zero_velocity_flag) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _n (round __n))
   (setq _sb_length (round __sb_length))
   (setq _source __source)
   (setq _sensor_model __sensor_model)
   (setq _type __type)
   (setq _obs_info __obs_info)
   (setq _acceleration_x (float __acceleration_x))
   (setq _acceleration_y (float __acceleration_y))
   (setq _acceleration_z (float __acceleration_z))
   (setq _angular_rate_x (float __angular_rate_x))
   (setq _angular_rate_y (float __angular_rate_y))
   (setq _angular_rate_z (float __angular_rate_z))
   (setq _velocity_x (float __velocity_x))
   (setq _velocity_y (float __velocity_y))
   (setq _velocity_z (float __velocity_z))
   (setq _std_dev_x (float __std_dev_x))
   (setq _std_dev_y (float __std_dev_y))
   (setq _std_dev_z (float __std_dev_z))
   (setq _sensor_temperature (float __sensor_temperature))
   (setq _zero_velocity_flag (float __zero_velocity_flag))
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
  (:n
   (&optional __n)
   (if __n (setq _n __n)) _n)
  (:sb_length
   (&optional __sb_length)
   (if __sb_length (setq _sb_length __sb_length)) _sb_length)
  (:source
   (&optional __source)
   (if __source (setq _source __source)) _source)
  (:sensor_model
   (&optional __sensor_model)
   (if __sensor_model (setq _sensor_model __sensor_model)) _sensor_model)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:obs_info
   (&optional __obs_info)
   (if __obs_info (setq _obs_info __obs_info)) _obs_info)
  (:acceleration_x
   (&optional __acceleration_x)
   (if __acceleration_x (setq _acceleration_x __acceleration_x)) _acceleration_x)
  (:acceleration_y
   (&optional __acceleration_y)
   (if __acceleration_y (setq _acceleration_y __acceleration_y)) _acceleration_y)
  (:acceleration_z
   (&optional __acceleration_z)
   (if __acceleration_z (setq _acceleration_z __acceleration_z)) _acceleration_z)
  (:angular_rate_x
   (&optional __angular_rate_x)
   (if __angular_rate_x (setq _angular_rate_x __angular_rate_x)) _angular_rate_x)
  (:angular_rate_y
   (&optional __angular_rate_y)
   (if __angular_rate_y (setq _angular_rate_y __angular_rate_y)) _angular_rate_y)
  (:angular_rate_z
   (&optional __angular_rate_z)
   (if __angular_rate_z (setq _angular_rate_z __angular_rate_z)) _angular_rate_z)
  (:velocity_x
   (&optional __velocity_x)
   (if __velocity_x (setq _velocity_x __velocity_x)) _velocity_x)
  (:velocity_y
   (&optional __velocity_y)
   (if __velocity_y (setq _velocity_y __velocity_y)) _velocity_y)
  (:velocity_z
   (&optional __velocity_z)
   (if __velocity_z (setq _velocity_z __velocity_z)) _velocity_z)
  (:std_dev_x
   (&optional __std_dev_x)
   (if __std_dev_x (setq _std_dev_x __std_dev_x)) _std_dev_x)
  (:std_dev_y
   (&optional __std_dev_y)
   (if __std_dev_y (setq _std_dev_y __std_dev_y)) _std_dev_y)
  (:std_dev_z
   (&optional __std_dev_z)
   (if __std_dev_z (setq _std_dev_z __std_dev_z)) _std_dev_z)
  (:sensor_temperature
   (&optional __sensor_temperature)
   (if __sensor_temperature (setq _sensor_temperature __sensor_temperature)) _sensor_temperature)
  (:zero_velocity_flag
   (&optional __zero_velocity_flag)
   (if __zero_velocity_flag (setq _zero_velocity_flag __zero_velocity_flag)) _zero_velocity_flag)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; septentrio_gnss_driver/BlockHeader _block_header
    (send _block_header :serialization-length)
    ;; uint8 _n
    1
    ;; uint8 _sb_length
    1
    ;; uint8[] _source
    (* 1    (length _source)) 4
    ;; uint8[] _sensor_model
    (* 1    (length _sensor_model)) 4
    ;; uint8[] _type
    (* 1    (length _type)) 4
    ;; uint8[] _obs_info
    (* 1    (length _obs_info)) 4
    ;; float64 _acceleration_x
    8
    ;; float64 _acceleration_y
    8
    ;; float64 _acceleration_z
    8
    ;; float64 _angular_rate_x
    8
    ;; float64 _angular_rate_y
    8
    ;; float64 _angular_rate_z
    8
    ;; float32 _velocity_x
    4
    ;; float32 _velocity_y
    4
    ;; float32 _velocity_z
    4
    ;; float32 _std_dev_x
    4
    ;; float32 _std_dev_y
    4
    ;; float32 _std_dev_z
    4
    ;; float32 _sensor_temperature
    4
    ;; float64 _zero_velocity_flag
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; septentrio_gnss_driver/BlockHeader _block_header
       (send _block_header :serialize s)
     ;; uint8 _n
       (write-byte _n s)
     ;; uint8 _sb_length
       (write-byte _sb_length s)
     ;; uint8[] _source
     (write-long (length _source) s)
     (princ _source s)
     ;; uint8[] _sensor_model
     (write-long (length _sensor_model) s)
     (princ _sensor_model s)
     ;; uint8[] _type
     (write-long (length _type) s)
     (princ _type s)
     ;; uint8[] _obs_info
     (write-long (length _obs_info) s)
     (princ _obs_info s)
     ;; float64 _acceleration_x
       (sys::poke _acceleration_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _acceleration_y
       (sys::poke _acceleration_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _acceleration_z
       (sys::poke _acceleration_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angular_rate_x
       (sys::poke _angular_rate_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angular_rate_y
       (sys::poke _angular_rate_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angular_rate_z
       (sys::poke _angular_rate_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _velocity_x
       (sys::poke _velocity_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity_y
       (sys::poke _velocity_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity_z
       (sys::poke _velocity_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _std_dev_x
       (sys::poke _std_dev_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _std_dev_y
       (sys::poke _std_dev_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _std_dev_z
       (sys::poke _std_dev_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _sensor_temperature
       (sys::poke _sensor_temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _zero_velocity_flag
       (sys::poke _zero_velocity_flag (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; septentrio_gnss_driver/BlockHeader _block_header
     (send _block_header :deserialize buf ptr-) (incf ptr- (send _block_header :serialization-length))
   ;; uint8 _n
     (setq _n (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sb_length
     (setq _sb_length (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _source
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _source (make-array n :element-type :char))
     (replace _source buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _sensor_model
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _sensor_model (make-array n :element-type :char))
     (replace _sensor_model buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _type
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _type (make-array n :element-type :char))
     (replace _type buf :start2 ptr-) (incf ptr- n))
   ;; uint8[] _obs_info
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _obs_info (make-array n :element-type :char))
     (replace _obs_info buf :start2 ptr-) (incf ptr- n))
   ;; float64 _acceleration_x
     (setq _acceleration_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _acceleration_y
     (setq _acceleration_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _acceleration_z
     (setq _acceleration_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angular_rate_x
     (setq _angular_rate_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angular_rate_y
     (setq _angular_rate_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angular_rate_z
     (setq _angular_rate_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _velocity_x
     (setq _velocity_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity_y
     (setq _velocity_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity_z
     (setq _velocity_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _std_dev_x
     (setq _std_dev_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _std_dev_y
     (setq _std_dev_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _std_dev_z
     (setq _std_dev_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _sensor_temperature
     (setq _sensor_temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _zero_velocity_flag
     (setq _zero_velocity_flag (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::ExtSensorMeas :md5sum-) "bc4155d6dac83ebf51ac1aba432000ae")
(setf (get septentrio_gnss_driver::ExtSensorMeas :datatype-) "septentrio_gnss_driver/ExtSensorMeas")
(setf (get septentrio_gnss_driver::ExtSensorMeas :definition-)
      "# ExtSensorMeas block 
# Block_Number 4050

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint8   n
uint8   sb_length

# ExtSensorMeasSet
uint8[]   source
uint8[]   sensor_model
uint8[]   type
uint8[]   obs_info

#ExtSensorMeasAcceleration
float64 acceleration_x # m/s^2
float64 acceleration_y # m/s^2
float64 acceleration_z # m/s^2

#ExtSensorMeasAngularRate
float64 angular_rate_x # deg/s
float64 angular_rate_y # deg/s
float64 angular_rate_z # deg/s

#ExtSensorMeasVelocity
float32 velocity_x # m/s
float32 velocity_y # m/s
float32 velocity_z # m/s
float32 std_dev_x # m/s
float32 std_dev_y # m/s
float32 std_dev_z # m/s

#ExtSensorMeasInfo
float32 sensor_temperature # deg C

#ExtSensorMeasZeroVelocityFlag
float64 zero_velocity_flag
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



(provide :septentrio_gnss_driver/ExtSensorMeas "bc4155d6dac83ebf51ac1aba432000ae")


