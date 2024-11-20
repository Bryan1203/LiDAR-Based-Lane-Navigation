; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude PosCovCartesian.msg.html

(cl:defclass <PosCovCartesian> (roslisp-msg-protocol:ros-message)
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
   (cov_xx
    :reader cov_xx
    :initarg :cov_xx
    :type cl:float
    :initform 0.0)
   (cov_yy
    :reader cov_yy
    :initarg :cov_yy
    :type cl:float
    :initform 0.0)
   (cov_zz
    :reader cov_zz
    :initarg :cov_zz
    :type cl:float
    :initform 0.0)
   (cov_bb
    :reader cov_bb
    :initarg :cov_bb
    :type cl:float
    :initform 0.0)
   (cov_xy
    :reader cov_xy
    :initarg :cov_xy
    :type cl:float
    :initform 0.0)
   (cov_xz
    :reader cov_xz
    :initarg :cov_xz
    :type cl:float
    :initform 0.0)
   (cov_xb
    :reader cov_xb
    :initarg :cov_xb
    :type cl:float
    :initform 0.0)
   (cov_yz
    :reader cov_yz
    :initarg :cov_yz
    :type cl:float
    :initform 0.0)
   (cov_yb
    :reader cov_yb
    :initarg :cov_yb
    :type cl:float
    :initform 0.0)
   (cov_zb
    :reader cov_zb
    :initarg :cov_zb
    :type cl:float
    :initform 0.0))
)

(cl:defclass PosCovCartesian (<PosCovCartesian>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PosCovCartesian>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PosCovCartesian)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<PosCovCartesian> is deprecated: use septentrio_gnss_driver-msg:PosCovCartesian instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:mode-val is deprecated.  Use septentrio_gnss_driver-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:error-val is deprecated.  Use septentrio_gnss_driver-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'cov_xx-val :lambda-list '(m))
(cl:defmethod cov_xx-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_xx-val is deprecated.  Use septentrio_gnss_driver-msg:cov_xx instead.")
  (cov_xx m))

(cl:ensure-generic-function 'cov_yy-val :lambda-list '(m))
(cl:defmethod cov_yy-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_yy-val is deprecated.  Use septentrio_gnss_driver-msg:cov_yy instead.")
  (cov_yy m))

(cl:ensure-generic-function 'cov_zz-val :lambda-list '(m))
(cl:defmethod cov_zz-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_zz-val is deprecated.  Use septentrio_gnss_driver-msg:cov_zz instead.")
  (cov_zz m))

(cl:ensure-generic-function 'cov_bb-val :lambda-list '(m))
(cl:defmethod cov_bb-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_bb-val is deprecated.  Use septentrio_gnss_driver-msg:cov_bb instead.")
  (cov_bb m))

(cl:ensure-generic-function 'cov_xy-val :lambda-list '(m))
(cl:defmethod cov_xy-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_xy-val is deprecated.  Use septentrio_gnss_driver-msg:cov_xy instead.")
  (cov_xy m))

(cl:ensure-generic-function 'cov_xz-val :lambda-list '(m))
(cl:defmethod cov_xz-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_xz-val is deprecated.  Use septentrio_gnss_driver-msg:cov_xz instead.")
  (cov_xz m))

(cl:ensure-generic-function 'cov_xb-val :lambda-list '(m))
(cl:defmethod cov_xb-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_xb-val is deprecated.  Use septentrio_gnss_driver-msg:cov_xb instead.")
  (cov_xb m))

(cl:ensure-generic-function 'cov_yz-val :lambda-list '(m))
(cl:defmethod cov_yz-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_yz-val is deprecated.  Use septentrio_gnss_driver-msg:cov_yz instead.")
  (cov_yz m))

(cl:ensure-generic-function 'cov_yb-val :lambda-list '(m))
(cl:defmethod cov_yb-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_yb-val is deprecated.  Use septentrio_gnss_driver-msg:cov_yb instead.")
  (cov_yb m))

(cl:ensure-generic-function 'cov_zb-val :lambda-list '(m))
(cl:defmethod cov_zb-val ((m <PosCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_zb-val is deprecated.  Use septentrio_gnss_driver-msg:cov_zb instead.")
  (cov_zb m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PosCovCartesian>) ostream)
  "Serializes a message object of type '<PosCovCartesian>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_xx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_yy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_zz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_bb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_xy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_xz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_xb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_yz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_yb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_zb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PosCovCartesian>) istream)
  "Deserializes a message object of type '<PosCovCartesian>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_xx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_yy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_zz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_bb) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_xy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_xz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_xb) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_yz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_yb) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_zb) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PosCovCartesian>)))
  "Returns string type for a message object of type '<PosCovCartesian>"
  "septentrio_gnss_driver/PosCovCartesian")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PosCovCartesian)))
  "Returns string type for a message object of type 'PosCovCartesian"
  "septentrio_gnss_driver/PosCovCartesian")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PosCovCartesian>)))
  "Returns md5sum for a message object of type '<PosCovCartesian>"
  "6f9c694efd927f0282537b2693bfa92f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PosCovCartesian)))
  "Returns md5sum for a message object of type 'PosCovCartesian"
  "6f9c694efd927f0282537b2693bfa92f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PosCovCartesian>)))
  "Returns full string definition for message of type '<PosCovCartesian>"
  (cl:format cl:nil "# PVTGeodetic block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        mode~%uint8        error        ~%float32      cov_xx # m^2~%float32      cov_yy # m^2~%float32      cov_zz # m^2~%float32      cov_bb # m^2~%float32      cov_xy # m^2~%float32      cov_xz # m^2~%float32      cov_xb # m^2~%float32      cov_yz # m^2~%float32      cov_yb # m^2~%float32      cov_zb # m^2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PosCovCartesian)))
  "Returns full string definition for message of type 'PosCovCartesian"
  (cl:format cl:nil "# PVTGeodetic block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        mode~%uint8        error        ~%float32      cov_xx # m^2~%float32      cov_yy # m^2~%float32      cov_zz # m^2~%float32      cov_bb # m^2~%float32      cov_xy # m^2~%float32      cov_xz # m^2~%float32      cov_xb # m^2~%float32      cov_yz # m^2~%float32      cov_yb # m^2~%float32      cov_zb # m^2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PosCovCartesian>))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PosCovCartesian>))
  "Converts a ROS message object to a list"
  (cl:list 'PosCovCartesian
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':error (error msg))
    (cl:cons ':cov_xx (cov_xx msg))
    (cl:cons ':cov_yy (cov_yy msg))
    (cl:cons ':cov_zz (cov_zz msg))
    (cl:cons ':cov_bb (cov_bb msg))
    (cl:cons ':cov_xy (cov_xy msg))
    (cl:cons ':cov_xz (cov_xz msg))
    (cl:cons ':cov_xb (cov_xb msg))
    (cl:cons ':cov_yz (cov_yz msg))
    (cl:cons ':cov_yb (cov_yb msg))
    (cl:cons ':cov_zb (cov_zb msg))
))
