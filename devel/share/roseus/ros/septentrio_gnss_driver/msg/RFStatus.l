;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::RFStatus)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'RFStatus (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::RFSTATUS")
  (make-package "SEPTENTRIO_GNSS_DRIVER::RFSTATUS"))

(in-package "ROS")
;;//! \htmlinclude RFStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::RFStatus
  :super ros::object
  :slots (_header _block_header _n _sb_length _flags _rfband ))

(defmethod septentrio_gnss_driver::RFStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:n __n) 0)
    ((:sb_length __sb_length) 0)
    ((:flags __flags) 0)
    ((:rfband __rfband) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _n (round __n))
   (setq _sb_length (round __sb_length))
   (setq _flags (round __flags))
   (setq _rfband __rfband)
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
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:rfband
   (&rest __rfband)
   (if (keywordp (car __rfband))
       (send* _rfband __rfband)
     (progn
       (if __rfband (setq _rfband (car __rfband)))
       _rfband)))
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
    ;; uint8 _flags
    1
    ;; septentrio_gnss_driver/RFBand[] _rfband
    (apply #'+ (send-all _rfband :serialization-length)) 4
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
     ;; uint8 _flags
       (write-byte _flags s)
     ;; septentrio_gnss_driver/RFBand[] _rfband
     (write-long (length _rfband) s)
     (dolist (elem _rfband)
       (send elem :serialize s)
       )
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
   ;; uint8 _flags
     (setq _flags (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; septentrio_gnss_driver/RFBand[] _rfband
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rfband (let (r) (dotimes (i n) (push (instance septentrio_gnss_driver::RFBand :init) r)) r))
     (dolist (elem- _rfband)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::RFStatus :md5sum-) "6223e703baf4d3da19e391be72a94d34")
(setf (get septentrio_gnss_driver::RFStatus :datatype-) "septentrio_gnss_driver/RFStatus")
(setf (get septentrio_gnss_driver::RFStatus :definition-)
      "# RFStatus block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

uint8    n
uint8    sb_length
uint8    flags
RFBand[] rfband


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
================================================================================
MSG: septentrio_gnss_driver/RFBand
# RFband block

uint32 frequency # Hz
uint16 bandwidth # kHz
uint8  info
")



(provide :septentrio_gnss_driver/RFStatus "6223e703baf4d3da19e391be72a94d34")


