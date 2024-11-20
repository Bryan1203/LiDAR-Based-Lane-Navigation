;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::PosCovGeodetic)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'PosCovGeodetic (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::POSCOVGEODETIC")
  (make-package "SEPTENTRIO_GNSS_DRIVER::POSCOVGEODETIC"))

(in-package "ROS")
;;//! \htmlinclude PosCovGeodetic.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::PosCovGeodetic
  :super ros::object
  :slots (_header _block_header _mode _error _cov_latlat _cov_lonlon _cov_hgthgt _cov_bb _cov_latlon _cov_lathgt _cov_latb _cov_lonhgt _cov_lonb _cov_hb ))

(defmethod septentrio_gnss_driver::PosCovGeodetic
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:mode __mode) 0)
    ((:error __error) 0)
    ((:cov_latlat __cov_latlat) 0.0)
    ((:cov_lonlon __cov_lonlon) 0.0)
    ((:cov_hgthgt __cov_hgthgt) 0.0)
    ((:cov_bb __cov_bb) 0.0)
    ((:cov_latlon __cov_latlon) 0.0)
    ((:cov_lathgt __cov_lathgt) 0.0)
    ((:cov_latb __cov_latb) 0.0)
    ((:cov_lonhgt __cov_lonhgt) 0.0)
    ((:cov_lonb __cov_lonb) 0.0)
    ((:cov_hb __cov_hb) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _mode (round __mode))
   (setq _error (round __error))
   (setq _cov_latlat (float __cov_latlat))
   (setq _cov_lonlon (float __cov_lonlon))
   (setq _cov_hgthgt (float __cov_hgthgt))
   (setq _cov_bb (float __cov_bb))
   (setq _cov_latlon (float __cov_latlon))
   (setq _cov_lathgt (float __cov_lathgt))
   (setq _cov_latb (float __cov_latb))
   (setq _cov_lonhgt (float __cov_lonhgt))
   (setq _cov_lonb (float __cov_lonb))
   (setq _cov_hb (float __cov_hb))
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
  (:cov_latlat
   (&optional __cov_latlat)
   (if __cov_latlat (setq _cov_latlat __cov_latlat)) _cov_latlat)
  (:cov_lonlon
   (&optional __cov_lonlon)
   (if __cov_lonlon (setq _cov_lonlon __cov_lonlon)) _cov_lonlon)
  (:cov_hgthgt
   (&optional __cov_hgthgt)
   (if __cov_hgthgt (setq _cov_hgthgt __cov_hgthgt)) _cov_hgthgt)
  (:cov_bb
   (&optional __cov_bb)
   (if __cov_bb (setq _cov_bb __cov_bb)) _cov_bb)
  (:cov_latlon
   (&optional __cov_latlon)
   (if __cov_latlon (setq _cov_latlon __cov_latlon)) _cov_latlon)
  (:cov_lathgt
   (&optional __cov_lathgt)
   (if __cov_lathgt (setq _cov_lathgt __cov_lathgt)) _cov_lathgt)
  (:cov_latb
   (&optional __cov_latb)
   (if __cov_latb (setq _cov_latb __cov_latb)) _cov_latb)
  (:cov_lonhgt
   (&optional __cov_lonhgt)
   (if __cov_lonhgt (setq _cov_lonhgt __cov_lonhgt)) _cov_lonhgt)
  (:cov_lonb
   (&optional __cov_lonb)
   (if __cov_lonb (setq _cov_lonb __cov_lonb)) _cov_lonb)
  (:cov_hb
   (&optional __cov_hb)
   (if __cov_hb (setq _cov_hb __cov_hb)) _cov_hb)
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
    ;; float32 _cov_latlat
    4
    ;; float32 _cov_lonlon
    4
    ;; float32 _cov_hgthgt
    4
    ;; float32 _cov_bb
    4
    ;; float32 _cov_latlon
    4
    ;; float32 _cov_lathgt
    4
    ;; float32 _cov_latb
    4
    ;; float32 _cov_lonhgt
    4
    ;; float32 _cov_lonb
    4
    ;; float32 _cov_hb
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
     ;; float32 _cov_latlat
       (sys::poke _cov_latlat (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_lonlon
       (sys::poke _cov_lonlon (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_hgthgt
       (sys::poke _cov_hgthgt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_bb
       (sys::poke _cov_bb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_latlon
       (sys::poke _cov_latlon (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_lathgt
       (sys::poke _cov_lathgt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_latb
       (sys::poke _cov_latb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_lonhgt
       (sys::poke _cov_lonhgt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_lonb
       (sys::poke _cov_lonb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_hb
       (sys::poke _cov_hb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _cov_latlat
     (setq _cov_latlat (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_lonlon
     (setq _cov_lonlon (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_hgthgt
     (setq _cov_hgthgt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_bb
     (setq _cov_bb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_latlon
     (setq _cov_latlon (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_lathgt
     (setq _cov_lathgt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_latb
     (setq _cov_latb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_lonhgt
     (setq _cov_lonhgt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_lonb
     (setq _cov_lonb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_hb
     (setq _cov_hb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::PosCovGeodetic :md5sum-) "d5a0ad571ba33ff67cc97dcff03d6b87")
(setf (get septentrio_gnss_driver::PosCovGeodetic :datatype-) "septentrio_gnss_driver/PosCovGeodetic")
(setf (get septentrio_gnss_driver::PosCovGeodetic :definition-)
      "# PVTGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_latlat # m^2
float32      cov_lonlon # m^2
float32      cov_hgthgt # m^2
float32      cov_bb     # m^2
float32      cov_latlon # m^2
float32      cov_lathgt # m^2
float32      cov_latb   # m^2
float32      cov_lonhgt # m^2
float32      cov_lonb   # m^2
float32      cov_hb     # m^2
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



(provide :septentrio_gnss_driver/PosCovGeodetic "d5a0ad571ba33ff67cc97dcff03d6b87")


