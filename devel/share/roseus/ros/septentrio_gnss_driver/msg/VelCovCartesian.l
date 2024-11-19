;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::VelCovCartesian)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'VelCovCartesian (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::VELCOVCARTESIAN")
  (make-package "SEPTENTRIO_GNSS_DRIVER::VELCOVCARTESIAN"))

(in-package "ROS")
;;//! \htmlinclude VelCovCartesian.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::VelCovCartesian
  :super ros::object
  :slots (_header _block_header _mode _error _cov_vxvx _cov_vyvy _cov_vzvz _cov_dtdt _cov_vxvy _cov_vxvz _cov_vxdt _cov_vyvz _cov_vydt _cov_vzdt ))

(defmethod septentrio_gnss_driver::VelCovCartesian
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:mode __mode) 0)
    ((:error __error) 0)
    ((:cov_vxvx __cov_vxvx) 0.0)
    ((:cov_vyvy __cov_vyvy) 0.0)
    ((:cov_vzvz __cov_vzvz) 0.0)
    ((:cov_dtdt __cov_dtdt) 0.0)
    ((:cov_vxvy __cov_vxvy) 0.0)
    ((:cov_vxvz __cov_vxvz) 0.0)
    ((:cov_vxdt __cov_vxdt) 0.0)
    ((:cov_vyvz __cov_vyvz) 0.0)
    ((:cov_vydt __cov_vydt) 0.0)
    ((:cov_vzdt __cov_vzdt) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _mode (round __mode))
   (setq _error (round __error))
   (setq _cov_vxvx (float __cov_vxvx))
   (setq _cov_vyvy (float __cov_vyvy))
   (setq _cov_vzvz (float __cov_vzvz))
   (setq _cov_dtdt (float __cov_dtdt))
   (setq _cov_vxvy (float __cov_vxvy))
   (setq _cov_vxvz (float __cov_vxvz))
   (setq _cov_vxdt (float __cov_vxdt))
   (setq _cov_vyvz (float __cov_vyvz))
   (setq _cov_vydt (float __cov_vydt))
   (setq _cov_vzdt (float __cov_vzdt))
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
  (:cov_vxvx
   (&optional __cov_vxvx)
   (if __cov_vxvx (setq _cov_vxvx __cov_vxvx)) _cov_vxvx)
  (:cov_vyvy
   (&optional __cov_vyvy)
   (if __cov_vyvy (setq _cov_vyvy __cov_vyvy)) _cov_vyvy)
  (:cov_vzvz
   (&optional __cov_vzvz)
   (if __cov_vzvz (setq _cov_vzvz __cov_vzvz)) _cov_vzvz)
  (:cov_dtdt
   (&optional __cov_dtdt)
   (if __cov_dtdt (setq _cov_dtdt __cov_dtdt)) _cov_dtdt)
  (:cov_vxvy
   (&optional __cov_vxvy)
   (if __cov_vxvy (setq _cov_vxvy __cov_vxvy)) _cov_vxvy)
  (:cov_vxvz
   (&optional __cov_vxvz)
   (if __cov_vxvz (setq _cov_vxvz __cov_vxvz)) _cov_vxvz)
  (:cov_vxdt
   (&optional __cov_vxdt)
   (if __cov_vxdt (setq _cov_vxdt __cov_vxdt)) _cov_vxdt)
  (:cov_vyvz
   (&optional __cov_vyvz)
   (if __cov_vyvz (setq _cov_vyvz __cov_vyvz)) _cov_vyvz)
  (:cov_vydt
   (&optional __cov_vydt)
   (if __cov_vydt (setq _cov_vydt __cov_vydt)) _cov_vydt)
  (:cov_vzdt
   (&optional __cov_vzdt)
   (if __cov_vzdt (setq _cov_vzdt __cov_vzdt)) _cov_vzdt)
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
    ;; float32 _cov_vxvx
    4
    ;; float32 _cov_vyvy
    4
    ;; float32 _cov_vzvz
    4
    ;; float32 _cov_dtdt
    4
    ;; float32 _cov_vxvy
    4
    ;; float32 _cov_vxvz
    4
    ;; float32 _cov_vxdt
    4
    ;; float32 _cov_vyvz
    4
    ;; float32 _cov_vydt
    4
    ;; float32 _cov_vzdt
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
     ;; float32 _cov_vxvx
       (sys::poke _cov_vxvx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vyvy
       (sys::poke _cov_vyvy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vzvz
       (sys::poke _cov_vzvz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_dtdt
       (sys::poke _cov_dtdt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vxvy
       (sys::poke _cov_vxvy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vxvz
       (sys::poke _cov_vxvz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vxdt
       (sys::poke _cov_vxdt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vyvz
       (sys::poke _cov_vyvz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vydt
       (sys::poke _cov_vydt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cov_vzdt
       (sys::poke _cov_vzdt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _cov_vxvx
     (setq _cov_vxvx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vyvy
     (setq _cov_vyvy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vzvz
     (setq _cov_vzvz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_dtdt
     (setq _cov_dtdt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vxvy
     (setq _cov_vxvy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vxvz
     (setq _cov_vxvz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vxdt
     (setq _cov_vxdt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vyvz
     (setq _cov_vyvz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vydt
     (setq _cov_vydt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cov_vzdt
     (setq _cov_vzdt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::VelCovCartesian :md5sum-) "0675e2a2bdddd5904100644aa4bf90a3")
(setf (get septentrio_gnss_driver::VelCovCartesian :datatype-) "septentrio_gnss_driver/VelCovCartesian")
(setf (get septentrio_gnss_driver::VelCovCartesian :definition-)
      "# VelCovCartesian block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float32      cov_vxvx # m^2/s^2
float32      cov_vyvy # m^2/s^2
float32      cov_vzvz # m^2/s^2
float32      cov_dtdt # m^2/s^2
float32      cov_vxvy # m^2/s^2
float32      cov_vxvz # m^2/s^2
float32      cov_vxdt # m^2/s^2
float32      cov_vyvz # m^2/s^2
float32      cov_vydt # m^2/s^2
float32      cov_vzdt # m^2/s^2
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



(provide :septentrio_gnss_driver/VelCovCartesian "0675e2a2bdddd5904100644aa4bf90a3")


