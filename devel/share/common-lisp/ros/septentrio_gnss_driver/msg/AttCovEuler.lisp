; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude AttCovEuler.msg.html

(cl:defclass <AttCovEuler> (roslisp-msg-protocol:ros-message)
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
   (error
    :reader error
    :initarg :error
    :type cl:fixnum
    :initform 0)
   (cov_headhead
    :reader cov_headhead
    :initarg :cov_headhead
    :type cl:float
    :initform 0.0)
   (cov_pitchpitch
    :reader cov_pitchpitch
    :initarg :cov_pitchpitch
    :type cl:float
    :initform 0.0)
   (cov_rollroll
    :reader cov_rollroll
    :initarg :cov_rollroll
    :type cl:float
    :initform 0.0)
   (cov_headpitch
    :reader cov_headpitch
    :initarg :cov_headpitch
    :type cl:float
    :initform 0.0)
   (cov_headroll
    :reader cov_headroll
    :initarg :cov_headroll
    :type cl:float
    :initform 0.0)
   (cov_pitchroll
    :reader cov_pitchroll
    :initarg :cov_pitchroll
    :type cl:float
    :initform 0.0))
)

(cl:defclass AttCovEuler (<AttCovEuler>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AttCovEuler>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AttCovEuler)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<AttCovEuler> is deprecated: use septentrio_gnss_driver-msg:AttCovEuler instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AttCovEuler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <AttCovEuler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <AttCovEuler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:error-val is deprecated.  Use septentrio_gnss_driver-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'cov_headhead-val :lambda-list '(m))
(cl:defmethod cov_headhead-val ((m <AttCovEuler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_headhead-val is deprecated.  Use septentrio_gnss_driver-msg:cov_headhead instead.")
  (cov_headhead m))

(cl:ensure-generic-function 'cov_pitchpitch-val :lambda-list '(m))
(cl:defmethod cov_pitchpitch-val ((m <AttCovEuler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_pitchpitch-val is deprecated.  Use septentrio_gnss_driver-msg:cov_pitchpitch instead.")
  (cov_pitchpitch m))

(cl:ensure-generic-function 'cov_rollroll-val :lambda-list '(m))
(cl:defmethod cov_rollroll-val ((m <AttCovEuler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_rollroll-val is deprecated.  Use septentrio_gnss_driver-msg:cov_rollroll instead.")
  (cov_rollroll m))

(cl:ensure-generic-function 'cov_headpitch-val :lambda-list '(m))
(cl:defmethod cov_headpitch-val ((m <AttCovEuler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_headpitch-val is deprecated.  Use septentrio_gnss_driver-msg:cov_headpitch instead.")
  (cov_headpitch m))

(cl:ensure-generic-function 'cov_headroll-val :lambda-list '(m))
(cl:defmethod cov_headroll-val ((m <AttCovEuler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_headroll-val is deprecated.  Use septentrio_gnss_driver-msg:cov_headroll instead.")
  (cov_headroll m))

(cl:ensure-generic-function 'cov_pitchroll-val :lambda-list '(m))
(cl:defmethod cov_pitchroll-val ((m <AttCovEuler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_pitchroll-val is deprecated.  Use septentrio_gnss_driver-msg:cov_pitchroll instead.")
  (cov_pitchroll m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AttCovEuler>) ostream)
  "Serializes a message object of type '<AttCovEuler>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_headhead))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_pitchpitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_rollroll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_headpitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_headroll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_pitchroll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AttCovEuler>) istream)
  "Deserializes a message object of type '<AttCovEuler>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_headhead) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_pitchpitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_rollroll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_headpitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_headroll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_pitchroll) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AttCovEuler>)))
  "Returns string type for a message object of type '<AttCovEuler>"
  "septentrio_gnss_driver/AttCovEuler")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AttCovEuler)))
  "Returns string type for a message object of type 'AttCovEuler"
  "septentrio_gnss_driver/AttCovEuler")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AttCovEuler>)))
  "Returns md5sum for a message object of type '<AttCovEuler>"
  "e0afc6c2dfb1f98f719a573ace215ea7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AttCovEuler)))
  "Returns md5sum for a message object of type 'AttCovEuler"
  "e0afc6c2dfb1f98f719a573ace215ea7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AttCovEuler>)))
  "Returns full string definition for message of type '<AttCovEuler>"
  (cl:format cl:nil "# AttCovEuler block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        error   ~%float32      cov_headhead   # deg^2~%float32      cov_pitchpitch # deg^2~%float32      cov_rollroll   # deg^2 ~%float32      cov_headpitch  # deg^2~%float32      cov_headroll   # deg^2~%float32      cov_pitchroll  # deg^2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AttCovEuler)))
  "Returns full string definition for message of type 'AttCovEuler"
  (cl:format cl:nil "# AttCovEuler block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        error   ~%float32      cov_headhead   # deg^2~%float32      cov_pitchpitch # deg^2~%float32      cov_rollroll   # deg^2 ~%float32      cov_headpitch  # deg^2~%float32      cov_headroll   # deg^2~%float32      cov_pitchroll  # deg^2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AttCovEuler>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'block_header))
     1
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AttCovEuler>))
  "Converts a ROS message object to a list"
  (cl:list 'AttCovEuler
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':error (error msg))
    (cl:cons ':cov_headhead (cov_headhead msg))
    (cl:cons ':cov_pitchpitch (cov_pitchpitch msg))
    (cl:cons ':cov_rollroll (cov_rollroll msg))
    (cl:cons ':cov_headpitch (cov_headpitch msg))
    (cl:cons ':cov_headroll (cov_headroll msg))
    (cl:cons ':cov_pitchroll (cov_pitchroll msg))
))
