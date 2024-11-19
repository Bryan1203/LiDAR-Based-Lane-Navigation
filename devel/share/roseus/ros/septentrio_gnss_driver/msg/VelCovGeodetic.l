;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::VelCovGeodetic)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'VelCovGeodetic (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::VELCOVGEODETIC")
  (make-package "SEPTENTRIO_GNSS_DRIVER::VELCOVGEODETIC"))

(in-package "ROS")
;;//! \htmlinclude VelCovGeodetic.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::VelCovGeodetic
  :super ros::object
  :slots (_header _block_header _mode _error _cov_vnvn _cov_veve _cov_vuvu _cov_dtdt _cov_vnve _cov_vnvu _cov_vndt _cov_vevu _cov_vedt _cov_vudt ))

(defmethod septentrio_gnss_driver::VelCovGeodetic
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:mode __mode) 0)
    ((:error __error) 0)
    ((:cov_vnvn __cov_vnvn) 0.0)
    ((:cov_veve __cov_veve) 0.0)
    ((:cov_vuvu __cov_vuvu) 0.0)
    ((:cov_dtdt __cov_dtdt) 0.0)
    ((:cov_vnve __cov_vnve) 0.0)
    ((:cov_vnvu __cov_vnvu) 0.0)
    ((:cov_vndt __cov_vndt) 0.0)
    ((:cov_vevu __cov_vevu) 0.0)
    ((:cov_vedt __cov_vedt) 0.0)
    ((:cov_vudt __cov_vudt) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _mode (round __mode))
   (setq _error (round __error))
   (setq _cov_vnvn (float __cov_vnvn))
   (setq _cov_veve (float __cov_veve))
   (setq _cov_vuvu (float __cov_vuvu))
   (setq _cov_dtdt (float __cov_dtdt))
   (setq _cov_vnve (float __cov_vnve))
   (setq _cov_vnvu (float __cov_vnvu))
   (setq _cov_vndt (float __cov_vndt))
   (setq _cov_vevu (float __cov_vevu))
   (setq _cov_vedt (float __cov_vedt))
   (setq _cov_vudt (float __cov_vudt))
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
  (:cov_vnvn
   (&optional __cov_vnvn)
   (if __cov_vnvn (setq _cov_vnvn __cov_vnvn)) _cov_vnvn)
  (:cov_veve
   (&optional __cov_veve)
   (if __cov_veve (setq _cov_veve __cov_veve)) _cov_veve)
  (:cov_vuvu
   (&optional __cov_vuvu)
   (if __cov_vuvu (setq _cov_vuvu __cov_vuvu)) _cov_vuvu)
  (:cov_dtdt
   (&optional __cov_dtdt)
   (if __cov_dtdt (setq _cov_dtdt __cov_dtdt)) _cov_dtdt)
  (:cov_vnve
   (&optional __cov_vnve)
   (if __cov_vnve (setq _cov_vnve __cov_vnve)) _cov_vnve)
  (:cov_vnvu
   (&optional __cov_vnvu)
   (if __cov_vnvu (setq _cov_vnvu __cov_vnvu)) _cov_vnvu)
  (:cov_vndt
   (&optional __cov_vndt)
   (if __cov_vndt (setq _cov_vndt __cov_vndt)) _cov_vndt)
  (:cov_vevu
   (&optional __cov_vevu)
   (if __cov_vevu (setq _cov_vevu __cov_vevu)) _cov_vevu)
  (:cov_vedt
   (&optional __cov_vedt)
   (if __cov_vedt (setq _cov_vedt __cov_vedt)) _cov_vedt)
  (:cov_vudt
   (&optional __cov_vudt)
   (if __cov_vudt (setq _cov_vudt __cov_vudt)) _cov_vudt)
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
    ;; float32 _cov_vnvn
    4
    ;; float32 _cov_veve
    4
    ;; float32 _cov_vuvu
    4
    ;; float32 _cov_dtdt
    4
    ;; float32 _cov_vnve
    4
    ;; float32 _cov_vnvu
    4
    ;; float32 _cov_vndt
    4
    ;; float32 _cov_vevu
    4
    ;; float32 _cov_vedt
    4
    ;; float32 _cov_vudt
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
     ;; float32 _cov_vnvn
       (sys::poke _cov_vnvn (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_veve
       (sys::poke _cov_veve (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vuvu
       (sys::poke _cov_vuvu (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_dtdt
       (sys::poke _cov_dtdt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vnve
       (sys::poke _cov_vnve (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vnvu
       (sys::poke _cov_vnvu (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vndt
       (sys::poke _cov_vndt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vevu
       (sys::poke _cov_vevu (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vedt
       (sys::poke _cov_vedt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vudt
       (sys::poke _cov_vudt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _cov_vnvn
     (setq _cov_vnvn (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_veve
     (setq _cov_veve (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vuvu
     (setq _cov_vuvu (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_dtdt
     (setq _cov_dtdt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vnve
     (setq _cov_vnve (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vnvu
     (setq _cov_vnvu (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vndt
     (setq _cov_vndt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vevu
     (setq _cov_vevu (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vedt
     (setq _cov_vedt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vudt
     (setq _cov_vudt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::VelCovGeodetic :md5sum-) "b62aa4ae504c44f18ac72015246a1586")
(setf (get septentrio_gnss_driver::VelCovGeodetic :datatype-) "septentrio_gnss_driver/VelCovGeodetic")
(setf (get septentrio_gnss_driver::VelCovGeodetic :definition-)
      "# VelCovGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_vnvn # m^2/s^2
float32      cov_veve # m^2/s^2
float32      cov_vuvu # m^2/s^2
float32      cov_dtdt # m^2/s^2
float32      cov_vnve # m^2/s^2
float32      cov_vnvu # m^2/s^2
float32      cov_vndt # m^2/s^2
float32      cov_vevu # m^2/s^2
float32      cov_vedt # m^2/s^2
float32      cov_vudt # m^2/s^2
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



(provide :septentrio_gnss_driver/VelCovGeodetic "b62aa4ae504c44f18ac72015246a1586")


