; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude GALAuthStatus.msg.html

(cl:defclass <GALAuthStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (block_header
    :reader block_header
    :initarg :block_header
    :type septentrio_gnss_driver-msg:BlockHeader
    :initform (cl:make-instance 'septentrio_gnss_driver-msg:BlockHeader))
   (osnma_status
    :reader osnma_status
    :initarg :osnma_status
    :type cl:fixnum
    :initform 0)
   (trusted_time_delta
    :reader trusted_time_delta
    :initarg :trusted_time_delta
    :type cl:float
    :initform 0.0)
   (gal_active_mask
    :reader gal_active_mask
    :initarg :gal_active_mask
    :type cl:integer
    :initform 0)
   (gal_authentic_mask
    :reader gal_authentic_mask
    :initarg :gal_authentic_mask
    :type cl:integer
    :initform 0)
   (gps_active_mask
    :reader gps_active_mask
    :initarg :gps_active_mask
    :type cl:integer
    :initform 0)
   (gps_authentic_mask
    :reader gps_authentic_mask
    :initarg :gps_authentic_mask
    :type cl:integer
    :initform 0))
)

(cl:defclass GALAuthStatus (<GALAuthStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GALAuthStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GALAuthStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<GALAuthStatus> is deprecated: use septentrio_gnss_driver-msg:GALAuthStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GALAuthStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <GALAuthStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'osnma_status-val :lambda-list '(m))
(cl:defmethod osnma_status-val ((m <GALAuthStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:osnma_status-val is deprecated.  Use septentrio_gnss_driver-msg:osnma_status instead.")
  (osnma_status m))

(cl:ensure-generic-function 'trusted_time_delta-val :lambda-list '(m))
(cl:defmethod trusted_time_delta-val ((m <GALAuthStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:trusted_time_delta-val is deprecated.  Use septentrio_gnss_driver-msg:trusted_time_delta instead.")
  (trusted_time_delta m))

(cl:ensure-generic-function 'gal_active_mask-val :lambda-list '(m))
(cl:defmethod gal_active_mask-val ((m <GALAuthStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:gal_active_mask-val is deprecated.  Use septentrio_gnss_driver-msg:gal_active_mask instead.")
  (gal_active_mask m))

(cl:ensure-generic-function 'gal_authentic_mask-val :lambda-list '(m))
(cl:defmethod gal_authentic_mask-val ((m <GALAuthStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:gal_authentic_mask-val is deprecated.  Use septentrio_gnss_driver-msg:gal_authentic_mask instead.")
  (gal_authentic_mask m))

(cl:ensure-generic-function 'gps_active_mask-val :lambda-list '(m))
(cl:defmethod gps_active_mask-val ((m <GALAuthStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:gps_active_mask-val is deprecated.  Use septentrio_gnss_driver-msg:gps_active_mask instead.")
  (gps_active_mask m))

(cl:ensure-generic-function 'gps_authentic_mask-val :lambda-list '(m))
(cl:defmethod gps_authentic_mask-val ((m <GALAuthStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:gps_authentic_mask-val is deprecated.  Use septentrio_gnss_driver-msg:gps_authentic_mask instead.")
  (gps_authentic_mask m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GALAuthStatus>) ostream)
  "Serializes a message object of type '<GALAuthStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'osnma_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'osnma_status)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'trusted_time_delta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gal_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gal_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gal_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gal_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'gal_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'gal_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'gal_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'gal_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gal_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gal_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gal_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gal_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'gal_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'gal_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'gal_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'gal_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gps_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gps_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gps_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'gps_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'gps_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'gps_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'gps_active_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gps_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gps_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gps_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'gps_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'gps_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'gps_authentic_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'gps_authentic_mask)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GALAuthStatus>) istream)
  "Deserializes a message object of type '<GALAuthStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'osnma_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'osnma_status)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'trusted_time_delta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gal_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gal_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gal_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gal_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'gal_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'gal_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'gal_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'gal_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gal_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gal_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gal_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gal_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'gal_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'gal_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'gal_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'gal_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gps_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gps_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gps_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'gps_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'gps_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'gps_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'gps_active_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gps_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gps_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gps_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'gps_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'gps_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'gps_authentic_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'gps_authentic_mask)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GALAuthStatus>)))
  "Returns string type for a message object of type '<GALAuthStatus>"
  "septentrio_gnss_driver/GALAuthStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GALAuthStatus)))
  "Returns string type for a message object of type 'GALAuthStatus"
  "septentrio_gnss_driver/GALAuthStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GALAuthStatus>)))
  "Returns md5sum for a message object of type '<GALAuthStatus>"
  "9b9a0d56668644a236059d089716150f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GALAuthStatus)))
  "Returns md5sum for a message object of type 'GALAuthStatus"
  "9b9a0d56668644a236059d089716150f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GALAuthStatus>)))
  "Returns full string definition for message of type '<GALAuthStatus>"
  (cl:format cl:nil "# GALAuthStatus block ~%# Block_Number 4245~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header~%~%uint16  osnma_status~%float32 trusted_time_delta # s~%uint64  gal_active_mask~%uint64  gal_authentic_mask~%uint64  gps_active_mask~%uint64  gps_authentic_mask~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GALAuthStatus)))
  "Returns full string definition for message of type 'GALAuthStatus"
  (cl:format cl:nil "# GALAuthStatus block ~%# Block_Number 4245~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header~%~%uint16  osnma_status~%float32 trusted_time_delta # s~%uint64  gal_active_mask~%uint64  gal_authentic_mask~%uint64  gps_active_mask~%uint64  gps_authentic_mask~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GALAuthStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'block_header))
     2
     4
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GALAuthStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'GALAuthStatus
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':osnma_status (osnma_status msg))
    (cl:cons ':trusted_time_delta (trusted_time_delta msg))
    (cl:cons ':gal_active_mask (gal_active_mask msg))
    (cl:cons ':gal_authentic_mask (gal_authentic_mask msg))
    (cl:cons ':gps_active_mask (gps_active_mask msg))
    (cl:cons ':gps_authentic_mask (gps_authentic_mask msg))
))
