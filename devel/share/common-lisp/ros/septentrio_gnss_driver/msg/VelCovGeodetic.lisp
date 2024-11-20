; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude VelCovGeodetic.msg.html

(cl:defclass <VelCovGeodetic> (roslisp-msg-protocol:ros-message)
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
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (error
    :reader error
    :initarg :error
    :type cl:fixnum
    :initform 0)
   (cov_vnvn
    :reader cov_vnvn
    :initarg :cov_vnvn
    :type cl:float
    :initform 0.0)
   (cov_veve
    :reader cov_veve
    :initarg :cov_veve
    :type cl:float
    :initform 0.0)
   (cov_vuvu
    :reader cov_vuvu
    :initarg :cov_vuvu
    :type cl:float
    :initform 0.0)
   (cov_dtdt
    :reader cov_dtdt
    :initarg :cov_dtdt
    :type cl:float
    :initform 0.0)
   (cov_vnve
    :reader cov_vnve
    :initarg :cov_vnve
    :type cl:float
    :initform 0.0)
   (cov_vnvu
    :reader cov_vnvu
    :initarg :cov_vnvu
    :type cl:float
    :initform 0.0)
   (cov_vndt
    :reader cov_vndt
    :initarg :cov_vndt
    :type cl:float
    :initform 0.0)
   (cov_vevu
    :reader cov_vevu
    :initarg :cov_vevu
    :type cl:float
    :initform 0.0)
   (cov_vedt
    :reader cov_vedt
    :initarg :cov_vedt
    :type cl:float
    :initform 0.0)
   (cov_vudt
    :reader cov_vudt
    :initarg :cov_vudt
    :type cl:float
    :initform 0.0))
)

(cl:defclass VelCovGeodetic (<VelCovGeodetic>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VelCovGeodetic>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VelCovGeodetic)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<VelCovGeodetic> is deprecated: use septentrio_gnss_driver-msg:VelCovGeodetic instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:mode-val is deprecated.  Use septentrio_gnss_driver-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:error-val is deprecated.  Use septentrio_gnss_driver-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'cov_vnvn-val :lambda-list '(m))
(cl:defmethod cov_vnvn-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vnvn-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vnvn instead.")
  (cov_vnvn m))

(cl:ensure-generic-function 'cov_veve-val :lambda-list '(m))
(cl:defmethod cov_veve-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_veve-val is deprecated.  Use septentrio_gnss_driver-msg:cov_veve instead.")
  (cov_veve m))

(cl:ensure-generic-function 'cov_vuvu-val :lambda-list '(m))
(cl:defmethod cov_vuvu-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vuvu-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vuvu instead.")
  (cov_vuvu m))

(cl:ensure-generic-function 'cov_dtdt-val :lambda-list '(m))
(cl:defmethod cov_dtdt-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_dtdt-val is deprecated.  Use septentrio_gnss_driver-msg:cov_dtdt instead.")
  (cov_dtdt m))

(cl:ensure-generic-function 'cov_vnve-val :lambda-list '(m))
(cl:defmethod cov_vnve-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vnve-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vnve instead.")
  (cov_vnve m))

(cl:ensure-generic-function 'cov_vnvu-val :lambda-list '(m))
(cl:defmethod cov_vnvu-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vnvu-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vnvu instead.")
  (cov_vnvu m))

(cl:ensure-generic-function 'cov_vndt-val :lambda-list '(m))
(cl:defmethod cov_vndt-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vndt-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vndt instead.")
  (cov_vndt m))

(cl:ensure-generic-function 'cov_vevu-val :lambda-list '(m))
(cl:defmethod cov_vevu-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vevu-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vevu instead.")
  (cov_vevu m))

(cl:ensure-generic-function 'cov_vedt-val :lambda-list '(m))
(cl:defmethod cov_vedt-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vedt-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vedt instead.")
  (cov_vedt m))

(cl:ensure-generic-function 'cov_vudt-val :lambda-list '(m))
(cl:defmethod cov_vudt-val ((m <VelCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vudt-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vudt instead.")
  (cov_vudt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VelCovGeodetic>) ostream)
  "Serializes a message object of type '<VelCovGeodetic>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vnvn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_veve))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vuvu))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_dtdt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vnve))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vnvu))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vndt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vevu))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vedt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vudt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VelCovGeodetic>) istream)
  "Deserializes a message object of type '<VelCovGeodetic>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vnvn) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_veve) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vuvu) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_dtdt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vnve) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vnvu) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vndt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vevu) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vedt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vudt) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VelCovGeodetic>)))
  "Returns string type for a message object of type '<VelCovGeodetic>"
  "septentrio_gnss_driver/VelCovGeodetic")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelCovGeodetic)))
  "Returns string type for a message object of type 'VelCovGeodetic"
  "septentrio_gnss_driver/VelCovGeodetic")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VelCovGeodetic>)))
  "Returns md5sum for a message object of type '<VelCovGeodetic>"
  "b62aa4ae504c44f18ac72015246a1586")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VelCovGeodetic)))
  "Returns md5sum for a message object of type 'VelCovGeodetic"
  "b62aa4ae504c44f18ac72015246a1586")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VelCovGeodetic>)))
  "Returns full string definition for message of type '<VelCovGeodetic>"
  (cl:format cl:nil "# VelCovGeodetic block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        mode~%uint8        error        ~%float32      cov_vnvn # m^2/s^2~%float32      cov_veve # m^2/s^2~%float32      cov_vuvu # m^2/s^2~%float32      cov_dtdt # m^2/s^2~%float32      cov_vnve # m^2/s^2~%float32      cov_vnvu # m^2/s^2~%float32      cov_vndt # m^2/s^2~%float32      cov_vevu # m^2/s^2~%float32      cov_vedt # m^2/s^2~%float32      cov_vudt # m^2/s^2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VelCovGeodetic)))
  "Returns full string definition for message of type 'VelCovGeodetic"
  (cl:format cl:nil "# VelCovGeodetic block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        mode~%uint8        error        ~%float32      cov_vnvn # m^2/s^2~%float32      cov_veve # m^2/s^2~%float32      cov_vuvu # m^2/s^2~%float32      cov_dtdt # m^2/s^2~%float32      cov_vnve # m^2/s^2~%float32      cov_vnvu # m^2/s^2~%float32      cov_vndt # m^2/s^2~%float32      cov_vevu # m^2/s^2~%float32      cov_vedt # m^2/s^2~%float32      cov_vudt # m^2/s^2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VelCovGeodetic>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'block_header))
     1
     1
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VelCovGeodetic>))
  "Converts a ROS message object to a list"
  (cl:list 'VelCovGeodetic
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':error (error msg))
    (cl:cons ':cov_vnvn (cov_vnvn msg))
    (cl:cons ':cov_veve (cov_veve msg))
    (cl:cons ':cov_vuvu (cov_vuvu msg))
    (cl:cons ':cov_dtdt (cov_dtdt msg))
    (cl:cons ':cov_vnve (cov_vnve msg))
    (cl:cons ':cov_vnvu (cov_vnvu msg))
    (cl:cons ':cov_vndt (cov_vndt msg))
    (cl:cons ':cov_vevu (cov_vevu msg))
    (cl:cons ':cov_vedt (cov_vedt msg))
    (cl:cons ':cov_vudt (cov_vudt msg))
))
