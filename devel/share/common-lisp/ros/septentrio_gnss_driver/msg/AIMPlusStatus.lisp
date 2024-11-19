; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude AIMPlusStatus.msg.html

(cl:defclass <AIMPlusStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (tow
    :reader tow
    :initarg :tow
    :type cl:integer
    :initform 0)
   (wnc
    :reader wnc
    :initarg :wnc
    :type cl:fixnum
    :initform 0)
   (interference
    :reader interference
    :initarg :interference
    :type cl:fixnum
    :initform 0)
   (spoofing
    :reader spoofing
    :initarg :spoofing
    :type cl:fixnum
    :initform 0)
   (osnma_authenticating
    :reader osnma_authenticating
    :initarg :osnma_authenticating
    :type cl:boolean
    :initform cl:nil)
   (galileo_authentic
    :reader galileo_authentic
    :initarg :galileo_authentic
    :type cl:fixnum
    :initform 0)
   (galileo_spoofed
    :reader galileo_spoofed
    :initarg :galileo_spoofed
    :type cl:fixnum
    :initform 0)
   (gps_authentic
    :reader gps_authentic
    :initarg :gps_authentic
    :type cl:fixnum
    :initform 0)
   (gps_spoofed
    :reader gps_spoofed
    :initarg :gps_spoofed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AIMPlusStatus (<AIMPlusStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AIMPlusStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AIMPlusStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<AIMPlusStatus> is deprecated: use septentrio_gnss_driver-msg:AIMPlusStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AIMPlusStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'tow-val :lambda-list '(m))
(cl:defmethod tow-val ((m <AIMPlusStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:tow-val is deprecated.  Use septentrio_gnss_driver-msg:tow instead.")
  (tow m))

(cl:ensure-generic-function 'wnc-val :lambda-list '(m))
(cl:defmethod wnc-val ((m <AIMPlusStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:wnc-val is deprecated.  Use septentrio_gnss_driver-msg:wnc instead.")
  (wnc m))

(cl:ensure-generic-function 'interference-val :lambda-list '(m))
(cl:defmethod interference-val ((m <AIMPlusStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:interference-val is deprecated.  Use septentrio_gnss_driver-msg:interference instead.")
  (interference m))

(cl:ensure-generic-function 'spoofing-val :lambda-list '(m))
(cl:defmethod spoofing-val ((m <AIMPlusStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:spoofing-val is deprecated.  Use septentrio_gnss_driver-msg:spoofing instead.")
  (spoofing m))

(cl:ensure-generic-function 'osnma_authenticating-val :lambda-list '(m))
(cl:defmethod osnma_authenticating-val ((m <AIMPlusStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:osnma_authenticating-val is deprecated.  Use septentrio_gnss_driver-msg:osnma_authenticating instead.")
  (osnma_authenticating m))

(cl:ensure-generic-function 'galileo_authentic-val :lambda-list '(m))
(cl:defmethod galileo_authentic-val ((m <AIMPlusStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:galileo_authentic-val is deprecated.  Use septentrio_gnss_driver-msg:galileo_authentic instead.")
  (galileo_authentic m))

(cl:ensure-generic-function 'galileo_spoofed-val :lambda-list '(m))
(cl:defmethod galileo_spoofed-val ((m <AIMPlusStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:galileo_spoofed-val is deprecated.  Use septentrio_gnss_driver-msg:galileo_spoofed instead.")
  (galileo_spoofed m))

(cl:ensure-generic-function 'gps_authentic-val :lambda-list '(m))
(cl:defmethod gps_authentic-val ((m <AIMPlusStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:gps_authentic-val is deprecated.  Use septentrio_gnss_driver-msg:gps_authentic instead.")
  (gps_authentic m))

(cl:ensure-generic-function 'gps_spoofed-val :lambda-list '(m))
(cl:defmethod gps_spoofed-val ((m <AIMPlusStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:gps_spoofed-val is deprecated.  Use septentrio_gnss_driver-msg:gps_spoofed instead.")
  (gps_spoofed m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AIMPlusStatus>)))
    "Constants for message type '<AIMPlusStatus>"
  '((:SPECTRUM_CLEAN . 0)
    (:INTERFERENCE_MITIGATED . 1)
    (:INTERFERENCE_PRESENT . 2)
    (:NONE_DETECTED . 0)
    (:SPOOFING_DETECTED_BY_OSNMA . 1)
    (:SPOOFING_DETECTED_BY_AUTHENTCITY_TEST . 2)
    (:SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AIMPlusStatus)))
    "Constants for message type 'AIMPlusStatus"
  '((:SPECTRUM_CLEAN . 0)
    (:INTERFERENCE_MITIGATED . 1)
    (:INTERFERENCE_PRESENT . 2)
    (:NONE_DETECTED . 0)
    (:SPOOFING_DETECTED_BY_OSNMA . 1)
    (:SPOOFING_DETECTED_BY_AUTHENTCITY_TEST . 2)
    (:SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AIMPlusStatus>) ostream)
  "Serializes a message object of type '<AIMPlusStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tow)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tow)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tow)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tow)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wnc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'wnc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'interference)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'spoofing)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'osnma_authenticating) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'galileo_authentic)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'galileo_spoofed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_authentic)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_spoofed)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AIMPlusStatus>) istream)
  "Deserializes a message object of type '<AIMPlusStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tow)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tow)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tow)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tow)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wnc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'wnc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'interference)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'spoofing)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'osnma_authenticating) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'galileo_authentic)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'galileo_spoofed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_authentic)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_spoofed)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AIMPlusStatus>)))
  "Returns string type for a message object of type '<AIMPlusStatus>"
  "septentrio_gnss_driver/AIMPlusStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AIMPlusStatus)))
  "Returns string type for a message object of type 'AIMPlusStatus"
  "septentrio_gnss_driver/AIMPlusStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AIMPlusStatus>)))
  "Returns md5sum for a message object of type '<AIMPlusStatus>"
  "85cf4fc2fc2598dca0b9bb856cb2dcdd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AIMPlusStatus)))
  "Returns md5sum for a message object of type 'AIMPlusStatus"
  "85cf4fc2fc2598dca0b9bb856cb2dcdd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AIMPlusStatus>)))
  "Returns full string definition for message of type '<AIMPlusStatus>"
  (cl:format cl:nil "# AIMPlusStatus message~%# ROS message header~%std_msgs/Header header~%~%uint32 tow # ms since week start~%uint16 wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%uint8 interference~%#-------------------------------~%uint8 SPECTRUM_CLEAN         = 0~%uint8 INTERFERENCE_MITIGATED = 1~%uint8 INTERFERENCE_PRESENT   = 2~%#-------------------------------~%~%uint8 spoofing~%#--------------------------------------------------------~%uint8 NONE_DETECTED                                   = 0~%uint8 SPOOFING_DETECTED_BY_OSNMA                      = 1~%uint8 SPOOFING_DETECTED_BY_AUTHENTCITY_TEST           = 2~%uint8 SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST = 3~%#--------------------------------------------------------~%~%bool  osnma_authenticating~%uint8 galileo_authentic~%uint8 galileo_spoofed~%uint8 gps_authentic~%uint8 gps_spoofed~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AIMPlusStatus)))
  "Returns full string definition for message of type 'AIMPlusStatus"
  (cl:format cl:nil "# AIMPlusStatus message~%# ROS message header~%std_msgs/Header header~%~%uint32 tow # ms since week start~%uint16 wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%uint8 interference~%#-------------------------------~%uint8 SPECTRUM_CLEAN         = 0~%uint8 INTERFERENCE_MITIGATED = 1~%uint8 INTERFERENCE_PRESENT   = 2~%#-------------------------------~%~%uint8 spoofing~%#--------------------------------------------------------~%uint8 NONE_DETECTED                                   = 0~%uint8 SPOOFING_DETECTED_BY_OSNMA                      = 1~%uint8 SPOOFING_DETECTED_BY_AUTHENTCITY_TEST           = 2~%uint8 SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST = 3~%#--------------------------------------------------------~%~%bool  osnma_authenticating~%uint8 galileo_authentic~%uint8 galileo_spoofed~%uint8 gps_authentic~%uint8 gps_spoofed~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AIMPlusStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     2
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AIMPlusStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'AIMPlusStatus
    (cl:cons ':header (header msg))
    (cl:cons ':tow (tow msg))
    (cl:cons ':wnc (wnc msg))
    (cl:cons ':interference (interference msg))
    (cl:cons ':spoofing (spoofing msg))
    (cl:cons ':osnma_authenticating (osnma_authenticating msg))
    (cl:cons ':galileo_authentic (galileo_authentic msg))
    (cl:cons ':galileo_spoofed (galileo_spoofed msg))
    (cl:cons ':gps_authentic (gps_authentic msg))
    (cl:cons ':gps_spoofed (gps_spoofed msg))
))
