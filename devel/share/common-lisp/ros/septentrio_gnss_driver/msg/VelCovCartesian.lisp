; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude VelCovCartesian.msg.html

(cl:defclass <VelCovCartesian> (roslisp-msg-protocol:ros-message)
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
   (cov_vxvx
    :reader cov_vxvx
    :initarg :cov_vxvx
    :type cl:float
    :initform 0.0)
   (cov_vyvy
    :reader cov_vyvy
    :initarg :cov_vyvy
    :type cl:float
    :initform 0.0)
   (cov_vzvz
    :reader cov_vzvz
    :initarg :cov_vzvz
    :type cl:float
    :initform 0.0)
   (cov_dtdt
    :reader cov_dtdt
    :initarg :cov_dtdt
    :type cl:float
    :initform 0.0)
   (cov_vxvy
    :reader cov_vxvy
    :initarg :cov_vxvy
    :type cl:float
    :initform 0.0)
   (cov_vxvz
    :reader cov_vxvz
    :initarg :cov_vxvz
    :type cl:float
    :initform 0.0)
   (cov_vxdt
    :reader cov_vxdt
    :initarg :cov_vxdt
    :type cl:float
    :initform 0.0)
   (cov_vyvz
    :reader cov_vyvz
    :initarg :cov_vyvz
    :type cl:float
    :initform 0.0)
   (cov_vydt
    :reader cov_vydt
    :initarg :cov_vydt
    :type cl:float
    :initform 0.0)
   (cov_vzdt
    :reader cov_vzdt
    :initarg :cov_vzdt
    :type cl:float
    :initform 0.0))
)

(cl:defclass VelCovCartesian (<VelCovCartesian>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VelCovCartesian>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VelCovCartesian)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<VelCovCartesian> is deprecated: use septentrio_gnss_driver-msg:VelCovCartesian instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:mode-val is deprecated.  Use septentrio_gnss_driver-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:error-val is deprecated.  Use septentrio_gnss_driver-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'cov_vxvx-val :lambda-list '(m))
(cl:defmethod cov_vxvx-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vxvx-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vxvx instead.")
  (cov_vxvx m))

(cl:ensure-generic-function 'cov_vyvy-val :lambda-list '(m))
(cl:defmethod cov_vyvy-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vyvy-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vyvy instead.")
  (cov_vyvy m))

(cl:ensure-generic-function 'cov_vzvz-val :lambda-list '(m))
(cl:defmethod cov_vzvz-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vzvz-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vzvz instead.")
  (cov_vzvz m))

(cl:ensure-generic-function 'cov_dtdt-val :lambda-list '(m))
(cl:defmethod cov_dtdt-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_dtdt-val is deprecated.  Use septentrio_gnss_driver-msg:cov_dtdt instead.")
  (cov_dtdt m))

(cl:ensure-generic-function 'cov_vxvy-val :lambda-list '(m))
(cl:defmethod cov_vxvy-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vxvy-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vxvy instead.")
  (cov_vxvy m))

(cl:ensure-generic-function 'cov_vxvz-val :lambda-list '(m))
(cl:defmethod cov_vxvz-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vxvz-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vxvz instead.")
  (cov_vxvz m))

(cl:ensure-generic-function 'cov_vxdt-val :lambda-list '(m))
(cl:defmethod cov_vxdt-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vxdt-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vxdt instead.")
  (cov_vxdt m))

(cl:ensure-generic-function 'cov_vyvz-val :lambda-list '(m))
(cl:defmethod cov_vyvz-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vyvz-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vyvz instead.")
  (cov_vyvz m))

(cl:ensure-generic-function 'cov_vydt-val :lambda-list '(m))
(cl:defmethod cov_vydt-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vydt-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vydt instead.")
  (cov_vydt m))

(cl:ensure-generic-function 'cov_vzdt-val :lambda-list '(m))
(cl:defmethod cov_vzdt-val ((m <VelCovCartesian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cov_vzdt-val is deprecated.  Use septentrio_gnss_driver-msg:cov_vzdt instead.")
  (cov_vzdt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VelCovCartesian>) ostream)
  "Serializes a message object of type '<VelCovCartesian>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vxvx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vyvy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vzvz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_dtdt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vxvy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vxvz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vxdt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vyvz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vydt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cov_vzdt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VelCovCartesian>) istream)
  "Deserializes a message object of type '<VelCovCartesian>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vxvx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vyvy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vzvz) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'cov_vxvy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vxvz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vxdt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vyvz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vydt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cov_vzdt) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VelCovCartesian>)))
  "Returns string type for a message object of type '<VelCovCartesian>"
  "septentrio_gnss_driver/VelCovCartesian")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelCovCartesian)))
  "Returns string type for a message object of type 'VelCovCartesian"
  "septentrio_gnss_driver/VelCovCartesian")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VelCovCartesian>)))
  "Returns md5sum for a message object of type '<VelCovCartesian>"
  "0675e2a2bdddd5904100644aa4bf90a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VelCovCartesian)))
  "Returns md5sum for a message object of type 'VelCovCartesian"
  "0675e2a2bdddd5904100644aa4bf90a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VelCovCartesian>)))
  "Returns full string definition for message of type '<VelCovCartesian>"
  (cl:format cl:nil "# VelCovCartesian block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        mode~%uint8        error        ~%float32      cov_vxvx # m^2/s^2~%float32      cov_vyvy # m^2/s^2~%float32      cov_vzvz # m^2/s^2~%float32      cov_dtdt # m^2/s^2~%float32      cov_vxvy # m^2/s^2~%float32      cov_vxvz # m^2/s^2~%float32      cov_vxdt # m^2/s^2~%float32      cov_vyvz # m^2/s^2~%float32      cov_vydt # m^2/s^2~%float32      cov_vzdt # m^2/s^2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VelCovCartesian)))
  "Returns full string definition for message of type 'VelCovCartesian"
  (cl:format cl:nil "# VelCovCartesian block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        mode~%uint8        error        ~%float32      cov_vxvx # m^2/s^2~%float32      cov_vyvy # m^2/s^2~%float32      cov_vzvz # m^2/s^2~%float32      cov_dtdt # m^2/s^2~%float32      cov_vxvy # m^2/s^2~%float32      cov_vxvz # m^2/s^2~%float32      cov_vxdt # m^2/s^2~%float32      cov_vyvz # m^2/s^2~%float32      cov_vydt # m^2/s^2~%float32      cov_vzdt # m^2/s^2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VelCovCartesian>))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VelCovCartesian>))
  "Converts a ROS message object to a list"
  (cl:list 'VelCovCartesian
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':error (error msg))
    (cl:cons ':cov_vxvx (cov_vxvx msg))
    (cl:cons ':cov_vyvy (cov_vyvy msg))
    (cl:cons ':cov_vzvz (cov_vzvz msg))
    (cl:cons ':cov_dtdt (cov_dtdt msg))
    (cl:cons ':cov_vxvy (cov_vxvy msg))
    (cl:cons ':cov_vxvz (cov_vxvz msg))
    (cl:cons ':cov_vxdt (cov_vxdt msg))
    (cl:cons ':cov_vyvz (cov_vyvz msg))
    (cl:cons ':cov_vydt (cov_vydt msg))
    (cl:cons ':cov_vzdt (cov_vzdt msg))
))
