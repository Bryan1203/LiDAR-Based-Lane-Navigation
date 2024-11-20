;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::AIMPlusStatus)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'AIMPlusStatus (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS")
  (make-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))

(in-package "ROS")
;;//! \htmlinclude AIMPlusStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*SPECTRUM_CLEAN*" (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(shadow '*SPECTRUM_CLEAN* (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(defconstant septentrio_gnss_driver::AIMPlusStatus::*SPECTRUM_CLEAN* 0)
(intern "*INTERFERENCE_MITIGATED*" (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(shadow '*INTERFERENCE_MITIGATED* (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(defconstant septentrio_gnss_driver::AIMPlusStatus::*INTERFERENCE_MITIGATED* 1)
(intern "*INTERFERENCE_PRESENT*" (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(shadow '*INTERFERENCE_PRESENT* (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(defconstant septentrio_gnss_driver::AIMPlusStatus::*INTERFERENCE_PRESENT* 2)
(intern "*NONE_DETECTED*" (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(shadow '*NONE_DETECTED* (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(defconstant septentrio_gnss_driver::AIMPlusStatus::*NONE_DETECTED* 0)
(intern "*SPOOFING_DETECTED_BY_OSNMA*" (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(shadow '*SPOOFING_DETECTED_BY_OSNMA* (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(defconstant septentrio_gnss_driver::AIMPlusStatus::*SPOOFING_DETECTED_BY_OSNMA* 1)
(intern "*SPOOFING_DETECTED_BY_AUTHENTCITY_TEST*" (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(shadow '*SPOOFING_DETECTED_BY_AUTHENTCITY_TEST* (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(defconstant septentrio_gnss_driver::AIMPlusStatus::*SPOOFING_DETECTED_BY_AUTHENTCITY_TEST* 2)
(intern "*SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST*" (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(shadow '*SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST* (find-package "SEPTENTRIO_GNSS_DRIVER::AIMPLUSSTATUS"))
(defconstant septentrio_gnss_driver::AIMPlusStatus::*SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST* 3)

(defun septentrio_gnss_driver::AIMPlusStatus-to-symbol (const)
  (cond
        ((= const 0) 'septentrio_gnss_driver::AIMPlusStatus::*SPECTRUM_CLEAN*)
        ((= const 1) 'septentrio_gnss_driver::AIMPlusStatus::*INTERFERENCE_MITIGATED*)
        ((= const 2) 'septentrio_gnss_driver::AIMPlusStatus::*INTERFERENCE_PRESENT*)
        ((= const 0) 'septentrio_gnss_driver::AIMPlusStatus::*NONE_DETECTED*)
        ((= const 1) 'septentrio_gnss_driver::AIMPlusStatus::*SPOOFING_DETECTED_BY_OSNMA*)
        ((= const 2) 'septentrio_gnss_driver::AIMPlusStatus::*SPOOFING_DETECTED_BY_AUTHENTCITY_TEST*)
        ((= const 3) 'septentrio_gnss_driver::AIMPlusStatus::*SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST*)
        (t nil)))

(defclass septentrio_gnss_driver::AIMPlusStatus
  :super ros::object
  :slots (_header _tow _wnc _interference _spoofing _osnma_authenticating _galileo_authentic _galileo_spoofed _gps_authentic _gps_spoofed ))

(defmethod septentrio_gnss_driver::AIMPlusStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:tow __tow) 0)
    ((:wnc __wnc) 0)
    ((:interference __interference) 0)
    ((:spoofing __spoofing) 0)
    ((:osnma_authenticating __osnma_authenticating) nil)
    ((:galileo_authentic __galileo_authentic) 0)
    ((:galileo_spoofed __galileo_spoofed) 0)
    ((:gps_authentic __gps_authentic) 0)
    ((:gps_spoofed __gps_spoofed) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _tow (round __tow))
   (setq _wnc (round __wnc))
   (setq _interference (round __interference))
   (setq _spoofing (round __spoofing))
   (setq _osnma_authenticating __osnma_authenticating)
   (setq _galileo_authentic (round __galileo_authentic))
   (setq _galileo_spoofed (round __galileo_spoofed))
   (setq _gps_authentic (round __gps_authentic))
   (setq _gps_spoofed (round __gps_spoofed))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:tow
   (&optional __tow)
   (if __tow (setq _tow __tow)) _tow)
  (:wnc
   (&optional __wnc)
   (if __wnc (setq _wnc __wnc)) _wnc)
  (:interference
   (&optional __interference)
   (if __interference (setq _interference __interference)) _interference)
  (:spoofing
   (&optional __spoofing)
   (if __spoofing (setq _spoofing __spoofing)) _spoofing)
  (:osnma_authenticating
   (&optional (__osnma_authenticating :null))
   (if (not (eq __osnma_authenticating :null)) (setq _osnma_authenticating __osnma_authenticating)) _osnma_authenticating)
  (:galileo_authentic
   (&optional __galileo_authentic)
   (if __galileo_authentic (setq _galileo_authentic __galileo_authentic)) _galileo_authentic)
  (:galileo_spoofed
   (&optional __galileo_spoofed)
   (if __galileo_spoofed (setq _galileo_spoofed __galileo_spoofed)) _galileo_spoofed)
  (:gps_authentic
   (&optional __gps_authentic)
   (if __gps_authentic (setq _gps_authentic __gps_authentic)) _gps_authentic)
  (:gps_spoofed
   (&optional __gps_spoofed)
   (if __gps_spoofed (setq _gps_spoofed __gps_spoofed)) _gps_spoofed)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _tow
    4
    ;; uint16 _wnc
    2
    ;; uint8 _interference
    1
    ;; uint8 _spoofing
    1
    ;; bool _osnma_authenticating
    1
    ;; uint8 _galileo_authentic
    1
    ;; uint8 _galileo_spoofed
    1
    ;; uint8 _gps_authentic
    1
    ;; uint8 _gps_spoofed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _tow
       (write-long _tow s)
     ;; uint16 _wnc
       (write-word _wnc s)
     ;; uint8 _interference
       (write-byte _interference s)
     ;; uint8 _spoofing
       (write-byte _spoofing s)
     ;; bool _osnma_authenticating
       (if _osnma_authenticating (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _galileo_authentic
       (write-byte _galileo_authentic s)
     ;; uint8 _galileo_spoofed
       (write-byte _galileo_spoofed s)
     ;; uint8 _gps_authentic
       (write-byte _gps_authentic s)
     ;; uint8 _gps_spoofed
       (write-byte _gps_spoofed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _tow
     (setq _tow (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _wnc
     (setq _wnc (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _interference
     (setq _interference (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _spoofing
     (setq _spoofing (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _osnma_authenticating
     (setq _osnma_authenticating (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _galileo_authentic
     (setq _galileo_authentic (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _galileo_spoofed
     (setq _galileo_spoofed (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gps_authentic
     (setq _gps_authentic (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gps_spoofed
     (setq _gps_spoofed (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::AIMPlusStatus :md5sum-) "85cf4fc2fc2598dca0b9bb856cb2dcdd")
(setf (get septentrio_gnss_driver::AIMPlusStatus :datatype-) "septentrio_gnss_driver/AIMPlusStatus")
(setf (get septentrio_gnss_driver::AIMPlusStatus :definition-)
      "# AIMPlusStatus message
# ROS message header
std_msgs/Header header

uint32 tow # ms since week start
uint16 wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     

uint8 interference
#-------------------------------
uint8 SPECTRUM_CLEAN         = 0
uint8 INTERFERENCE_MITIGATED = 1
uint8 INTERFERENCE_PRESENT   = 2
#-------------------------------

uint8 spoofing
#--------------------------------------------------------
uint8 NONE_DETECTED                                   = 0
uint8 SPOOFING_DETECTED_BY_OSNMA                      = 1
uint8 SPOOFING_DETECTED_BY_AUTHENTCITY_TEST           = 2
uint8 SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST = 3
#--------------------------------------------------------

bool  osnma_authenticating
uint8 galileo_authentic
uint8 galileo_spoofed
uint8 gps_authentic
uint8 gps_spoofed
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

")



(provide :septentrio_gnss_driver/AIMPlusStatus "85cf4fc2fc2598dca0b9bb856cb2dcdd")


