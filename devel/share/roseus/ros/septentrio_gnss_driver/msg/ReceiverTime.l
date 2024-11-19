;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::ReceiverTime)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'ReceiverTime (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::RECEIVERTIME")
  (make-package "SEPTENTRIO_GNSS_DRIVER::RECEIVERTIME"))

(in-package "ROS")
;;//! \htmlinclude ReceiverTime.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::ReceiverTime
  :super ros::object
  :slots (_header _block_header _utc_year _utc_month _utc_day _utc_hour _utc_min _utc_second _delta_ls _sync_level ))

(defmethod septentrio_gnss_driver::ReceiverTime
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:utc_year __utc_year) 0)
    ((:utc_month __utc_month) 0)
    ((:utc_day __utc_day) 0)
    ((:utc_hour __utc_hour) 0)
    ((:utc_min __utc_min) 0)
    ((:utc_second __utc_second) 0)
    ((:delta_ls __delta_ls) 0)
    ((:sync_level __sync_level) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _utc_year (round __utc_year))
   (setq _utc_month (round __utc_month))
   (setq _utc_day (round __utc_day))
   (setq _utc_hour (round __utc_hour))
   (setq _utc_min (round __utc_min))
   (setq _utc_second (round __utc_second))
   (setq _delta_ls (round __delta_ls))
   (setq _sync_level (round __sync_level))
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
  (:utc_year
   (&optional __utc_year)
   (if __utc_year (setq _utc_year __utc_year)) _utc_year)
  (:utc_month
   (&optional __utc_month)
   (if __utc_month (setq _utc_month __utc_month)) _utc_month)
  (:utc_day
   (&optional __utc_day)
   (if __utc_day (setq _utc_day __utc_day)) _utc_day)
  (:utc_hour
   (&optional __utc_hour)
   (if __utc_hour (setq _utc_hour __utc_hour)) _utc_hour)
  (:utc_min
   (&optional __utc_min)
   (if __utc_min (setq _utc_min __utc_min)) _utc_min)
  (:utc_second
   (&optional __utc_second)
   (if __utc_second (setq _utc_second __utc_second)) _utc_second)
  (:delta_ls
   (&optional __delta_ls)
   (if __delta_ls (setq _delta_ls __delta_ls)) _delta_ls)
  (:sync_level
   (&optional __sync_level)
   (if __sync_level (setq _sync_level __sync_level)) _sync_level)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; septentrio_gnss_driver/BlockHeader _block_header
    (send _block_header :serialization-length)
    ;; int8 _utc_year
    1
    ;; int8 _utc_month
    1
    ;; int8 _utc_day
    1
    ;; int8 _utc_hour
    1
    ;; int8 _utc_min
    1
    ;; int8 _utc_second
    1
    ;; int8 _delta_ls
    1
    ;; uint8 _sync_level
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; septentrio_gnss_driver/BlockHeader _block_header
       (send _block_header :serialize s)
     ;; int8 _utc_year
       (write-byte _utc_year s)
     ;; int8 _utc_month
       (write-byte _utc_month s)
     ;; int8 _utc_day
       (write-byte _utc_day s)
     ;; int8 _utc_hour
       (write-byte _utc_hour s)
     ;; int8 _utc_min
       (write-byte _utc_min s)
     ;; int8 _utc_second
       (write-byte _utc_second s)
     ;; int8 _delta_ls
       (write-byte _delta_ls s)
     ;; uint8 _sync_level
       (write-byte _sync_level s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; septentrio_gnss_driver/BlockHeader _block_header
     (send _block_header :deserialize buf ptr-) (incf ptr- (send _block_header :serialization-length))
   ;; int8 _utc_year
     (setq _utc_year (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _utc_year 127) (setq _utc_year (- _utc_year 256)))
   ;; int8 _utc_month
     (setq _utc_month (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _utc_month 127) (setq _utc_month (- _utc_month 256)))
   ;; int8 _utc_day
     (setq _utc_day (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _utc_day 127) (setq _utc_day (- _utc_day 256)))
   ;; int8 _utc_hour
     (setq _utc_hour (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _utc_hour 127) (setq _utc_hour (- _utc_hour 256)))
   ;; int8 _utc_min
     (setq _utc_min (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _utc_min 127) (setq _utc_min (- _utc_min 256)))
   ;; int8 _utc_second
     (setq _utc_second (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _utc_second 127) (setq _utc_second (- _utc_second 256)))
   ;; int8 _delta_ls
     (setq _delta_ls (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _delta_ls 127) (setq _delta_ls (- _delta_ls 256)))
   ;; uint8 _sync_level
     (setq _sync_level (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::ReceiverTime :md5sum-) "4e8efd4564fa50f687df91c6000b6c23")
(setf (get septentrio_gnss_driver::ReceiverTime :datatype-) "septentrio_gnss_driver/ReceiverTime")
(setf (get septentrio_gnss_driver::ReceiverTime :definition-)
      "# ReceiverTime block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header

int8  utc_year    # year
int8  utc_month   # month
int8  utc_day     # day
int8  utc_hour    # hour
int8  utc_min     # minute
int8  utc_second # s
int8  delta_ls   # s
uint8 sync_level
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



(provide :septentrio_gnss_driver/ReceiverTime "4e8efd4564fa50f687df91c6000b6c23")


