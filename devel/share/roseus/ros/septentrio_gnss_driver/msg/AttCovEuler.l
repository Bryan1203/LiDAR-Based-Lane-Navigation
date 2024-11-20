;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::AttCovEuler)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'AttCovEuler (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::ATTCOVEULER")
  (make-package "SEPTENTRIO_GNSS_DRIVER::ATTCOVEULER"))

(in-package "ROS")
;;//! \htmlinclude AttCovEuler.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::AttCovEuler
  :super ros::object
  :slots (_header _block_header _error _cov_headhead _cov_pitchpitch _cov_rollroll _cov_headpitch _cov_headroll _cov_pitchroll ))

(defmethod septentrio_gnss_driver::AttCovEuler
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:error __error) 0)
    ((:cov_headhead __cov_headhead) 0.0)
    ((:cov_pitchpitch __cov_pitchpitch) 0.0)
    ((:cov_rollroll __cov_rollroll) 0.0)
    ((:cov_headpitch __cov_headpitch) 0.0)
    ((:cov_headroll __cov_headroll) 0.0)
    ((:cov_pitchroll __cov_pitchroll) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _error (round __error))
   (setq _cov_headhead (float __cov_headhead))
   (setq _cov_pitchpitch (float __cov_pitchpitch))
   (setq _cov_rollroll (float __cov_rollroll))
   (setq _cov_headpitch (float __cov_headpitch))
   (setq _cov_headroll (float __cov_headroll))
   (setq _cov_pitchroll (float __cov_pitchroll))
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
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:cov_headhead
   (&optional __cov_headhead)
   (if __cov_headhead (setq _cov_headhead __cov_headhead)) _cov_headhead)
  (:cov_pitchpitch
   (&optional __cov_pitchpitch)
   (if __cov_pitchpitch (setq _cov_pitchpitch __cov_pitchpitch)) _cov_pitchpitch)
  (:cov_rollroll
   (&optional __cov_rollroll)
   (if __cov_rollroll (setq _cov_rollroll __cov_rollroll)) _cov_rollroll)
  (:cov_headpitch
   (&optional __cov_headpitch)
   (if __cov_headpitch (setq _cov_headpitch __cov_headpitch)) _cov_headpitch)
  (:cov_headroll
   (&optional __cov_headroll)
   (if __cov_headroll (setq _cov_headroll __cov_headroll)) _cov_headroll)
  (:cov_pitchroll
   (&optional __cov_pitchroll)
   (if __cov_pitchroll (setq _cov_pitchroll __cov_pitchroll)) _cov_pitchroll)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; septentrio_gnss_driver/BlockHeader _block_header
    (send _block_header :serialization-length)
    ;; uint8 _error
    1
    ;; float32 _cov_headhead
    4
    ;; float32 _cov_pitchpitch
    4
    ;; float32 _cov_rollroll
    4
    ;; float32 _cov_headpitch
    4
    ;; float32 _cov_headroll
    4
    ;; float32 _cov_pitchroll
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
     ;; uint8 _error
       (write-byte _error s)
     ;; float32 _cov_headhead
       (sys::poke _cov_headhead (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_pitchpitch
       (sys::poke _cov_pitchpitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_rollroll
       (sys::poke _cov_rollroll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_headpitch
       (sys::poke _cov_headpitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_headroll
       (sys::poke _cov_headroll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_pitchroll
       (sys::poke _cov_pitchroll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; septentrio_gnss_driver/BlockHeader _block_header
     (send _block_header :deserialize buf ptr-) (incf ptr- (send _block_header :serialization-length))
   ;; uint8 _error
     (setq _error (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _cov_headhead
     (setq _cov_headhead (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_pitchpitch
     (setq _cov_pitchpitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_rollroll
     (setq _cov_rollroll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_headpitch
     (setq _cov_headpitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_headroll
     (setq _cov_headroll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_pitchroll
     (setq _cov_pitchroll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::AttCovEuler :md5sum-) "e0afc6c2dfb1f98f719a573ace215ea7")
(setf (get septentrio_gnss_driver::AttCovEuler :datatype-) "septentrio_gnss_driver/AttCovEuler")
(setf (get septentrio_gnss_driver::AttCovEuler :definition-)
      "# AttCovEuler block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        error   
float32      cov_headhead   # deg^2
float32      cov_pitchpitch # deg^2
float32      cov_rollroll   # deg^2 
float32      cov_headpitch  # deg^2
float32      cov_headroll   # deg^2
float32      cov_pitchroll  # deg^2

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



(provide :septentrio_gnss_driver/AttCovEuler "e0afc6c2dfb1f98f719a573ace215ea7")


