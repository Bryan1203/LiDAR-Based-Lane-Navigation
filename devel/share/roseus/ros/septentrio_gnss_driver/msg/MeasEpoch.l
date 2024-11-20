;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::MeasEpoch)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'MeasEpoch (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::MEASEPOCH")
  (make-package "SEPTENTRIO_GNSS_DRIVER::MEASEPOCH"))

(in-package "ROS")
;;//! \htmlinclude MeasEpoch.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::MeasEpoch
  :super ros::object
  :slots (_header _block_header _n _sb1_length _sb2_length _common_flags _cum_clk_jumps _type1 ))

(defmethod septentrio_gnss_driver::MeasEpoch
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:n __n) 0)
    ((:sb1_length __sb1_length) 0)
    ((:sb2_length __sb2_length) 0)
    ((:common_flags __common_flags) 0)
    ((:cum_clk_jumps __cum_clk_jumps) 0)
    ((:type1 __type1) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _n (round __n))
   (setq _sb1_length (round __sb1_length))
   (setq _sb2_length (round __sb2_length))
   (setq _common_flags (round __common_flags))
   (setq _cum_clk_jumps (round __cum_clk_jumps))
   (setq _type1 __type1)
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
  (:sb1_length
   (&optional __sb1_length)
   (if __sb1_length (setq _sb1_length __sb1_length)) _sb1_length)
  (:sb2_length
   (&optional __sb2_length)
   (if __sb2_length (setq _sb2_length __sb2_length)) _sb2_length)
  (:common_flags
   (&optional __common_flags)
   (if __common_flags (setq _common_flags __common_flags)) _common_flags)
  (:cum_clk_jumps
   (&optional __cum_clk_jumps)
   (if __cum_clk_jumps (setq _cum_clk_jumps __cum_clk_jumps)) _cum_clk_jumps)
  (:type1
   (&rest __type1)
   (if (keywordp (car __type1))
       (send* _type1 __type1)
     (progn
       (if __type1 (setq _type1 (car __type1)))
       _type1)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; septentrio_gnss_driver/BlockHeader _block_header
    (send _block_header :serialization-length)
    ;; uint8 _n
    1
    ;; uint8 _sb1_length
    1
    ;; uint8 _sb2_length
    1
    ;; uint8 _common_flags
    1
    ;; uint8 _cum_clk_jumps
    1
    ;; septentrio_gnss_driver/MeasEpochChannelType1[] _type1
    (apply #'+ (send-all _type1 :serialization-length)) 4
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
     ;; uint8 _sb1_length
       (write-byte _sb1_length s)
     ;; uint8 _sb2_length
       (write-byte _sb2_length s)
     ;; uint8 _common_flags
       (write-byte _common_flags s)
     ;; uint8 _cum_clk_jumps
       (write-byte _cum_clk_jumps s)
     ;; septentrio_gnss_driver/MeasEpochChannelType1[] _type1
     (write-long (length _type1) s)
     (dolist (elem _type1)
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
   ;; uint8 _sb1_length
     (setq _sb1_length (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sb2_length
     (setq _sb2_length (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _common_flags
     (setq _common_flags (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cum_clk_jumps
     (setq _cum_clk_jumps (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; septentrio_gnss_driver/MeasEpochChannelType1[] _type1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _type1 (let (r) (dotimes (i n) (push (instance septentrio_gnss_driver::MeasEpochChannelType1 :init) r)) r))
     (dolist (elem- _type1)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::MeasEpoch :md5sum-) "4cf9a1d78edfe10d2a772f4a50dd0645")
(setf (get septentrio_gnss_driver::MeasEpoch :datatype-) "septentrio_gnss_driver/MeasEpoch")
(setf (get septentrio_gnss_driver::MeasEpoch :definition-)
      "# MeasEpoch block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8 n
uint8 sb1_length
uint8 sb2_length      
uint8 common_flags
uint8 cum_clk_jumps # 0.001 s

MeasEpochChannelType1[] type1
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
MSG: septentrio_gnss_driver/MeasEpochChannelType1
# MeasEpochChannelType1 block
   
uint8  rx_channel
uint8  type
uint8  sv_id
uint8  misc        # 4294967.296 m
uint32 code_lsb    # 0.001 m
int32  doppler     # 0.0001 Hz
uint16 carrier_lsb # 0.001 cycles
int8   carrier_msb # 65.536 cycles
uint8  cn0         # 0.25 dB-Hz
uint16 lock_time
uint8  obs_info
uint8  n2

MeasEpochChannelType2[] type2
================================================================================
MSG: septentrio_gnss_driver/MeasEpochChannelType2
# MeasEpochChannelType2 block
   
uint8  type
uint8  lock_time          # s
uint8  cn0                # 0.25 dB-Hz
uint8  offsets_msb        # 65.536 m or 65.536 Hz
int8   carrier_msb        # 65.536 cycles
uint8  obs_info
uint16 code_offset_lsb    # 0.001 m
uint16 carrier_lsb        # 0.001 cycles
uint16 doppler_offset_lsb # 0.0001 Hz
")



(provide :septentrio_gnss_driver/MeasEpoch "4cf9a1d78edfe10d2a772f4a50dd0645")


