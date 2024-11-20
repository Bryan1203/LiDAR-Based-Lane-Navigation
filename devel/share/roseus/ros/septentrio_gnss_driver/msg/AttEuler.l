;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::AttEuler)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'AttEuler (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::ATTEULER")
  (make-package "SEPTENTRIO_GNSS_DRIVER::ATTEULER"))

(in-package "ROS")
;;//! \htmlinclude AttEuler.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::AttEuler
  :super ros::object
  :slots (_header _block_header _nr_sv _error _mode _heading _pitch _roll _pitch_dot _roll_dot _heading_dot ))

(defmethod septentrio_gnss_driver::AttEuler
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:nr_sv __nr_sv) 0)
    ((:error __error) 0)
    ((:mode __mode) 0)
    ((:heading __heading) 0.0)
    ((:pitch __pitch) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch_dot __pitch_dot) 0.0)
    ((:roll_dot __roll_dot) 0.0)
    ((:heading_dot __heading_dot) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _nr_sv (round __nr_sv))
   (setq _error (round __error))
   (setq _mode (round __mode))
   (setq _heading (float __heading))
   (setq _pitch (float __pitch))
   (setq _roll (float __roll))
   (setq _pitch_dot (float __pitch_dot))
   (setq _roll_dot (float __roll_dot))
   (setq _heading_dot (float __heading_dot))
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
  (:nr_sv
   (&optional __nr_sv)
   (if __nr_sv (setq _nr_sv __nr_sv)) _nr_sv)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch_dot
   (&optional __pitch_dot)
   (if __pitch_dot (setq _pitch_dot __pitch_dot)) _pitch_dot)
  (:roll_dot
   (&optional __roll_dot)
   (if __roll_dot (setq _roll_dot __roll_dot)) _roll_dot)
  (:heading_dot
   (&optional __heading_dot)
   (if __heading_dot (setq _heading_dot __heading_dot)) _heading_dot)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; septentrio_gnss_driver/BlockHeader _block_header
    (send _block_header :serialization-length)
    ;; uint8 _nr_sv
    1
    ;; uint8 _error
    1
    ;; uint16 _mode
    2
    ;; float32 _heading
    4
    ;; float32 _pitch
    4
    ;; float32 _roll
    4
    ;; float32 _pitch_dot
    4
    ;; float32 _roll_dot
    4
    ;; float32 _heading_dot
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
     ;; uint8 _nr_sv
       (write-byte _nr_sv s)
     ;; uint8 _error
       (write-byte _error s)
     ;; uint16 _mode
       (write-word _mode s)
     ;; float32 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_dot
       (sys::poke _pitch_dot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_dot
       (sys::poke _roll_dot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_dot
       (sys::poke _heading_dot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; septentrio_gnss_driver/BlockHeader _block_header
     (send _block_header :deserialize buf ptr-) (incf ptr- (send _block_header :serialization-length))
   ;; uint8 _nr_sv
     (setq _nr_sv (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _error
     (setq _error (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _mode
     (setq _mode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _heading
     (setq _heading (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_dot
     (setq _pitch_dot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_dot
     (setq _roll_dot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_dot
     (setq _heading_dot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::AttEuler :md5sum-) "99d953843f049fd84269fcb833dbdaa6")
(setf (get septentrio_gnss_driver::AttEuler :datatype-) "septentrio_gnss_driver/AttEuler")
(setf (get septentrio_gnss_driver::AttEuler :definition-)
      "# AttEuler block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        nr_sv
uint8        error        
uint16       mode
float32      heading     # deg
float32      pitch       # deg
float32      roll        # deg
float32      pitch_dot   # deg/s
float32      roll_dot    # deg/s
float32      heading_dot # deg/s

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



(provide :septentrio_gnss_driver/AttEuler "99d953843f049fd84269fcb833dbdaa6")


