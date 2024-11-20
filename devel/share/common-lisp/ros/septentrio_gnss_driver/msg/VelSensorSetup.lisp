; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude VelSensorSetup.msg.html

(cl:defclass <VelSensorSetup> (roslisp-msg-protocol:ros-message)
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
   (port
    :reader port
    :initarg :port
    :type cl:fixnum
    :initform 0)
   (lever_arm_x
    :reader lever_arm_x
    :initarg :lever_arm_x
    :type cl:float
    :initform 0.0)
   (lever_arm_y
    :reader lever_arm_y
    :initarg :lever_arm_y
    :type cl:float
    :initform 0.0)
   (lever_arm_z
    :reader lever_arm_z
    :initarg :lever_arm_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass VelSensorSetup (<VelSensorSetup>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VelSensorSetup>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VelSensorSetup)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<VelSensorSetup> is deprecated: use septentrio_gnss_driver-msg:VelSensorSetup instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VelSensorSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <VelSensorSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <VelSensorSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:port-val is deprecated.  Use septentrio_gnss_driver-msg:port instead.")
  (port m))

(cl:ensure-generic-function 'lever_arm_x-val :lambda-list '(m))
(cl:defmethod lever_arm_x-val ((m <VelSensorSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:lever_arm_x-val is deprecated.  Use septentrio_gnss_driver-msg:lever_arm_x instead.")
  (lever_arm_x m))

(cl:ensure-generic-function 'lever_arm_y-val :lambda-list '(m))
(cl:defmethod lever_arm_y-val ((m <VelSensorSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:lever_arm_y-val is deprecated.  Use septentrio_gnss_driver-msg:lever_arm_y instead.")
  (lever_arm_y m))

(cl:ensure-generic-function 'lever_arm_z-val :lambda-list '(m))
(cl:defmethod lever_arm_z-val ((m <VelSensorSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:lever_arm_z-val is deprecated.  Use septentrio_gnss_driver-msg:lever_arm_z instead.")
  (lever_arm_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VelSensorSetup>) ostream)
  "Serializes a message object of type '<VelSensorSetup>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lever_arm_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lever_arm_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lever_arm_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VelSensorSetup>) istream)
  "Deserializes a message object of type '<VelSensorSetup>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lever_arm_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lever_arm_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lever_arm_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VelSensorSetup>)))
  "Returns string type for a message object of type '<VelSensorSetup>"
  "septentrio_gnss_driver/VelSensorSetup")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelSensorSetup)))
  "Returns string type for a message object of type 'VelSensorSetup"
  "septentrio_gnss_driver/VelSensorSetup")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VelSensorSetup>)))
  "Returns md5sum for a message object of type '<VelSensorSetup>"
  "de5f86a0756c4a0797c2dda3d121932f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VelSensorSetup)))
  "Returns md5sum for a message object of type 'VelSensorSetup"
  "de5f86a0756c4a0797c2dda3d121932f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VelSensorSetup>)))
  "Returns full string definition for message of type '<VelSensorSetup>"
  (cl:format cl:nil "# VelSensorSetup block~%# Block_Number 4244~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header~%~%uint8   port~%float32 lever_arm_x # m~%float32 lever_arm_y # m~%float32 lever_arm_z # m~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VelSensorSetup)))
  "Returns full string definition for message of type 'VelSensorSetup"
  (cl:format cl:nil "# VelSensorSetup block~%# Block_Number 4244~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header~%~%uint8   port~%float32 lever_arm_x # m~%float32 lever_arm_y # m~%float32 lever_arm_z # m~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VelSensorSetup>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'block_header))
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VelSensorSetup>))
  "Converts a ROS message object to a list"
  (cl:list 'VelSensorSetup
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':port (port msg))
    (cl:cons ':lever_arm_x (lever_arm_x msg))
    (cl:cons ':lever_arm_y (lever_arm_y msg))
    (cl:cons ':lever_arm_z (lever_arm_z msg))
))
