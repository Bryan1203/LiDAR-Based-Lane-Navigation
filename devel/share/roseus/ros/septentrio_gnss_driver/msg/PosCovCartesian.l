;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::PosCovCartesian)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'PosCovCartesian (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::POSCOVCARTESIAN")
  (make-package "SEPTENTRIO_GNSS_DRIVER::POSCOVCARTESIAN"))

(in-package "ROS")
;;//! \htmlinclude PosCovCartesian.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::PosCovCartesian
  :super ros::object
  :slots (_header _block_header _mode _error _cov_xx _cov_yy _cov_zz _cov_bb _cov_xy _cov_xz _cov_xb _cov_yz _cov_yb _cov_zb ))

(defmethod septentrio_gnss_driver::PosCovCartesian
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:mode __mode) 0)
    ((:error __error) 0)
    ((:cov_xx __cov_xx) 0.0)
    ((:cov_yy __cov_yy) 0.0)
    ((:cov_zz __cov_zz) 0.0)
    ((:cov_bb __cov_bb) 0.0)
    ((:cov_xy __cov_xy) 0.0)
    ((:cov_xz __cov_xz) 0.0)
    ((:cov_xb __cov_xb) 0.0)
    ((:cov_yz __cov_yz) 0.0)
    ((:cov_yb __cov_yb) 0.0)
    ((:cov_zb __cov_zb) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _mode (round __mode))
   (setq _error (round __error))
   (setq _cov_xx (float __cov_xx))
   (setq _cov_yy (float __cov_yy))
   (setq _cov_zz (float __cov_zz))
   (setq _cov_bb (float __cov_bb))
   (setq _cov_xy (float __cov_xy))
   (setq _cov_xz (float __cov_xz))
   (setq _cov_xb (float __cov_xb))
   (setq _cov_yz (float __cov_yz))
   (setq _cov_yb (float __cov_yb))
   (setq _cov_zb (float __cov_zb))
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
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:cov_xx
   (&optional __cov_xx)
   (if __cov_xx (setq _cov_xx __cov_xx)) _cov_xx)
  (:cov_yy
   (&optional __cov_yy)
   (if __cov_yy (setq _cov_yy __cov_yy)) _cov_yy)
  (:cov_zz
   (&optional __cov_zz)
   (if __cov_zz (setq _cov_zz __cov_zz)) _cov_zz)
  (:cov_bb
   (&optional __cov_bb)
   (if __cov_bb (setq _cov_bb __cov_bb)) _cov_bb)
  (:cov_xy
   (&optional __cov_xy)
   (if __cov_xy (setq _cov_xy __cov_xy)) _cov_xy)
  (:cov_xz
   (&optional __cov_xz)
   (if __cov_xz (setq _cov_xz __cov_xz)) _cov_xz)
  (:cov_xb
   (&optional __cov_xb)
   (if __cov_xb (setq _cov_xb __cov_xb)) _cov_xb)
  (:cov_yz
   (&optional __cov_yz)
   (if __cov_yz (setq _cov_yz __cov_yz)) _cov_yz)
  (:cov_yb
   (&optional __cov_yb)
   (if __cov_yb (setq _cov_yb __cov_yb)) _cov_yb)
  (:cov_zb
   (&optional __cov_zb)
   (if __cov_zb (setq _cov_zb __cov_zb)) _cov_zb)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; septentrio_gnss_driver/BlockHeader _block_header
    (send _block_header :serialization-length)
    ;; uint8 _mode
    1
    ;; uint8 _error
    1
    ;; float32 _cov_xx
    4
    ;; float32 _cov_yy
    4
    ;; float32 _cov_zz
    4
    ;; float32 _cov_bb
    4
    ;; float32 _cov_xy
    4
    ;; float32 _cov_xz
    4
    ;; float32 _cov_xb
    4
    ;; float32 _cov_yz
    4
    ;; float32 _cov_yb
    4
    ;; float32 _cov_zb
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
     ;; uint8 _mode
       (write-byte _mode s)
     ;; uint8 _error
       (write-byte _error s)
     ;; float32 _cov_xx
       (sys::poke _cov_xx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_yy
       (sys::poke _cov_yy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_zz
       (sys::poke _cov_zz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_bb
       (sys::poke _cov_bb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_xy
       (sys::poke _cov_xy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_xz
       (sys::poke _cov_xz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_xb
       (sys::poke _cov_xb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_yz
       (sys::poke _cov_yz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_yb
       (sys::poke _cov_yb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_zb
       (sys::poke _cov_zb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; septentrio_gnss_driver/BlockHeader _block_header
     (send _block_header :deserialize buf ptr-) (incf ptr- (send _block_header :serialization-length))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _error
     (setq _error (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _cov_xx
     (setq _cov_xx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_yy
     (setq _cov_yy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_zz
     (setq _cov_zz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_bb
     (setq _cov_bb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_xy
     (setq _cov_xy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_xz
     (setq _cov_xz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_xb
     (setq _cov_xb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_yz
     (setq _cov_yz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_yb
     (setq _cov_yb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_zb
     (setq _cov_zb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::PosCovCartesian :md5sum-) "6f9c694efd927f0282537b2693bfa92f")
(setf (get septentrio_gnss_driver::PosCovCartesian :datatype-) "septentrio_gnss_driver/PosCovCartesian")
(setf (get septentrio_gnss_driver::PosCovCartesian :definition-)
      "# PVTGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_xx # m^2
float32      cov_yy # m^2
float32      cov_zz # m^2
float32      cov_bb # m^2
float32      cov_xy # m^2
float32      cov_xz # m^2
float32      cov_xb # m^2
float32      cov_yz # m^2
float32      cov_yb # m^2
float32      cov_zb # m^2
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



(provide :septentrio_gnss_driver/PosCovCartesian "6f9c694efd927f0282537b2693bfa92f")


