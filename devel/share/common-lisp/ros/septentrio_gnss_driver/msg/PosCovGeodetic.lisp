; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude PosCovGeodetic.msg.html

(cl:defclass <PosCovGeodetic> (roslisp-msg-protocol:ros-message)
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
   (cov_latlat
    :reader cov_latlat
    :initarg :cov_latlat
    :type cl:float
    :initform 0.0)
   (cov_lonlon
    :reader cov_lonlon
    :initarg :cov_lonlon
    :type cl:float
    :initform 0.0)
   (cov_hgthgt
    :reader cov_hgthgt
    :initarg :cov_hgthgt
    :type cl:float
    :initform 0.0)
   (cov_bb
    :reader cov_bb
    :initarg :cov_bb
    :type cl:float
    :initform 0.0)
   (cov_latlon
    :reader cov_latlon
    :initarg :cov_latlon
    :type cl:float
    :initform 0.0)
   (cov_lathgt
    :reader cov_lathgt
    :initarg :cov_lathgt
    :type cl:float
    :initform 0.0)
   (cov_latb
    :reader cov_latb
    :initarg :cov_latb
    :type cl:float
    :initform 0.0)
   (cov_lonhgt
    :reader cov_lonhgt
    :initarg :cov_lonhgt
    :type cl:float
    :initform 0.0)
   (cov_lonb
    :reader cov_lonb
    :initarg :cov_lonb
    :type cl:float
    :initform 0.0)
   (cov_hb
    :reader cov_hb
    :initarg :cov_hb
    :type cl:float
    :initform 0.0))
)

(cl:defclass PosCovGeodetic (<PosCovGeodetic>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PosCovGeodetic>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PosCovGeodetic)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<PosCovGeodetic> is deprecated: use septentrio_gnss_driver-msg:PosCovGeodetic instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:mode-val is deprecated.  Use septentrio_gnss_driver-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:error-val is deprecated.  Use septentrio_gnss_driver-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'cov_latlat-val :lambda-list '(m))
(cl:defmethod cov_latlat-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_latlat-val is deprecated.  Use septentrio_gnss_driver-msg:cov_latlat instead.")
  (cov_latlat m))

(cl:ensure-generic-function 'cov_lonlon-val :lambda-list '(m))
(cl:defmethod cov_lonlon-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_lonlon-val is deprecated.  Use septentrio_gnss_driver-msg:cov_lonlon instead.")
  (cov_lonlon m))

(cl:ensure-generic-function 'cov_hgthgt-val :lambda-list '(m))
(cl:defmethod cov_hgthgt-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_hgthgt-val is deprecated.  Use septentrio_gnss_driver-msg:cov_hgthgt instead.")
  (cov_hgthgt m))

(cl:ensure-generic-function 'cov_bb-val :lambda-list '(m))
(cl:defmethod cov_bb-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_bb-val is deprecated.  Use septentrio_gnss_driver-msg:cov_bb instead.")
  (cov_bb m))

(cl:ensure-generic-function 'cov_latlon-val :lambda-list '(m))
(cl:defmethod cov_latlon-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_latlon-val is deprecated.  Use septentrio_gnss_driver-msg:cov_latlon instead.")
  (cov_latlon m))

(cl:ensure-generic-function 'cov_lathgt-val :lambda-list '(m))
(cl:defmethod cov_lathgt-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_lathgt-val is deprecated.  Use septentrio_gnss_driver-msg:cov_lathgt instead.")
  (cov_lathgt m))

(cl:ensure-generic-function 'cov_latb-val :lambda-list '(m))
(cl:defmethod cov_latb-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_latb-val is deprecated.  Use septentrio_gnss_driver-msg:cov_latb instead.")
  (cov_latb m))

(cl:ensure-generic-function 'cov_lonhgt-val :lambda-list '(m))
(cl:defmethod cov_lonhgt-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_lonhgt-val is deprecated.  Use septentrio_gnss_driver-msg:cov_lonhgt instead.")
  (cov_lonhgt m))

(cl:ensure-generic-function 'cov_lonb-val :lambda-list '(m))
(cl:defmethod cov_lonb-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_lonb-val is deprecated.  Use septentrio_gnss_driver-msg:cov_lonb instead.")
  (cov_lonb m))

(cl:ensure-generic-function 'cov_hb-val :lambda-list '(m))
(cl:defmethod cov_hb-val ((m <PosCovGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_hb-val is deprecated.  Use septentrio_gnss_driver-msg:cov_hb instead.")
  (cov_hb m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PosCovGeodetic>) ostream)
  "Serializes a message object of type '<PosCovGeodetic>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_latlat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_lonlon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_hgthgt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_bb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_latlon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_lathgt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_latb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_lonhgt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_lonb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_hb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PosCovGeodetic>) istream)
  "Deserializes a message object of type '<PosCovGeodetic>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_latlat) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_lonlon) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_hgthgt) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'cov_latlon) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_lathgt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_latb) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_lonhgt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_lonb) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_hb) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PosCovGeodetic>)))
  "Returns string type for a message object of type '<PosCovGeodetic>"
  "septentrio_gnss_driver/PosCovGeodetic")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PosCovGeodetic)))
  "Returns string type for a message object of type 'PosCovGeodetic"
  "septentrio_gnss_driver/PosCovGeodetic")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PosCovGeodetic>)))
  "Returns md5sum for a message object of type '<PosCovGeodetic>"
  "d5a0ad571ba33ff67cc97dcff03d6b87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PosCovGeodetic)))
  "Returns md5sum for a message object of type 'PosCovGeodetic"
  "d5a0ad571ba33ff67cc97dcff03d6b87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PosCovGeodetic>)))
  "Returns full string definition for message of type '<PosCovGeodetic>"
  (cl:format cl:nil "# PVTGeodetic block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        mode~%uint8        error        ~%float32      cov_latlat # m^2~%float32      cov_lonlon # m^2~%float32      cov_hgthgt # m^2~%float32      cov_bb     # m^2~%float32      cov_latlon # m^2~%float32      cov_lathgt # m^2~%float32      cov_latb   # m^2~%float32      cov_lonhgt # m^2~%float32      cov_lonb   # m^2~%float32      cov_hb     # m^2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PosCovGeodetic)))
  "Returns full string definition for message of type 'PosCovGeodetic"
  (cl:format cl:nil "# PVTGeodetic block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        mode~%uint8        error        ~%float32      cov_latlat # m^2~%float32      cov_lonlon # m^2~%float32      cov_hgthgt # m^2~%float32      cov_bb     # m^2~%float32      cov_latlon # m^2~%float32      cov_lathgt # m^2~%float32      cov_latb   # m^2~%float32      cov_lonhgt # m^2~%float32      cov_lonb   # m^2~%float32      cov_hb     # m^2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PosCovGeodetic>))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PosCovGeodetic>))
  "Converts a ROS message object to a list"
  (cl:list 'PosCovGeodetic
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':error (error msg))
    (cl:cons ':cov_latlat (cov_latlat msg))
    (cl:cons ':cov_lonlon (cov_lonlon msg))
    (cl:cons ':cov_hgthgt (cov_hgthgt msg))
    (cl:cons ':cov_bb (cov_bb msg))
    (cl:cons ':cov_latlon (cov_latlon msg))
    (cl:cons ':cov_lathgt (cov_lathgt msg))
    (cl:cons ':cov_latb (cov_latb msg))
    (cl:cons ':cov_lonhgt (cov_lonhgt msg))
    (cl:cons ':cov_lonb (cov_lonb msg))
    (cl:cons ':cov_hb (cov_hb msg))
))
