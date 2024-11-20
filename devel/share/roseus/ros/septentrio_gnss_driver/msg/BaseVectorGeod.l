;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::BaseVectorGeod)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'BaseVectorGeod (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::BASEVECTORGEOD")
  (make-package "SEPTENTRIO_GNSS_DRIVER::BASEVECTORGEOD"))

(in-package "ROS")
;;//! \htmlinclude BaseVectorGeod.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::BaseVectorGeod
  :super ros::object
  :slots (_header _block_header _n _sb_length _vector_info_geod ))

(defmethod septentrio_gnss_driver::BaseVectorGeod
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:n __n) 0)
    ((:sb_length __sb_length) 0)
    ((:vector_info_geod __vector_info_geod) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _n (round __n))
   (setq _sb_length (round __sb_length))
   (setq _vector_info_geod __vector_info_geod)
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
  (:vector_info_geod
   (&rest __vector_info_geod)
   (if (keywordp (car __vector_info_geod))
       (send* _vector_info_geod __vector_info_geod)
     (progn
       (if __vector_info_geod (setq _vector_info_geod (car __vector_info_geod)))
       _vector_info_geod)))
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
    ;; septentrio_gnss_driver/VectorInfoGeod[] _vector_info_geod
    (apply #'+ (send-all _vector_info_geod :serialization-length)) 4
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
     ;; septentrio_gnss_driver/VectorInfoGeod[] _vector_info_geod
     (write-long (length _vector_info_geod) s)
     (dolist (elem _vector_info_geod)
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
   ;; septentrio_gnss_driver/VectorInfoGeod[] _vector_info_geod
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vector_info_geod (let (r) (dotimes (i n) (push (instance septentrio_gnss_driver::VectorInfoGeod :init) r)) r))
     (dolist (elem- _vector_info_geod)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::BaseVectorGeod :md5sum-) "1349428be748ae82d66f928c5daf71dd")
(setf (get septentrio_gnss_driver::BaseVectorGeod :datatype-) "septentrio_gnss_driver/BaseVectorGeod")
(setf (get septentrio_gnss_driver::BaseVectorGeod :definition-)
      "# BaseVectorGeod block 
# Block_Number 4028

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint8   n
uint8   sb_length

VectorInfoGeod[] vector_info_geod 
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
MSG: septentrio_gnss_driver/VectorInfoGeod
# VectorInfoGeod block
   
uint8   nr_sv
uint8   error
uint8   mode
uint8   misc
float64 delta_east   # m
float64 delta_north  # m
float64 delta_up     # m
float32 delta_ve     # m
float32 delta_vn     # m
float32 delta_vu     # m
uint16  azimuth      # 0.01 deg
int16   elevation    # 0.01 deg
uint16  reference_id
uint16  corr_age     # 0.01 s
uint32  signal_info
")



(provide :septentrio_gnss_driver/BaseVectorGeod "1349428be748ae82d66f928c5daf71dd")


