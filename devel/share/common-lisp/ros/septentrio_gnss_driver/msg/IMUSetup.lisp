; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude IMUSetup.msg.html

(cl:defclass <IMUSetup> (roslisp-msg-protocol:ros-message)
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
   (serial_port
    :reader serial_port
    :initarg :serial_port
    :type cl:fixnum
    :initform 0)
   (ant_lever_arm_x
    :reader ant_lever_arm_x
    :initarg :ant_lever_arm_x
    :type cl:float
    :initform 0.0)
   (ant_lever_arm_y
    :reader ant_lever_arm_y
    :initarg :ant_lever_arm_y
    :type cl:float
    :initform 0.0)
   (ant_lever_arm_z
    :reader ant_lever_arm_z
    :initarg :ant_lever_arm_z
    :type cl:float
    :initform 0.0)
   (theta_x
    :reader theta_x
    :initarg :theta_x
    :type cl:float
    :initform 0.0)
   (theta_y
    :reader theta_y
    :initarg :theta_y
    :type cl:float
    :initform 0.0)
   (theta_z
    :reader theta_z
    :initarg :theta_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass IMUSetup (<IMUSetup>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IMUSetup>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IMUSetup)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<IMUSetup> is deprecated: use septentrio_gnss_driver-msg:IMUSetup instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <IMUSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <IMUSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'serial_port-val :lambda-list '(m))
(cl:defmethod serial_port-val ((m <IMUSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:serial_port-val is deprecated.  Use septentrio_gnss_driver-msg:serial_port instead.")
  (serial_port m))

(cl:ensure-generic-function 'ant_lever_arm_x-val :lambda-list '(m))
(cl:defmethod ant_lever_arm_x-val ((m <IMUSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:ant_lever_arm_x-val is deprecated.  Use septentrio_gnss_driver-msg:ant_lever_arm_x instead.")
  (ant_lever_arm_x m))

(cl:ensure-generic-function 'ant_lever_arm_y-val :lambda-list '(m))
(cl:defmethod ant_lever_arm_y-val ((m <IMUSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:ant_lever_arm_y-val is deprecated.  Use septentrio_gnss_driver-msg:ant_lever_arm_y instead.")
  (ant_lever_arm_y m))

(cl:ensure-generic-function 'ant_lever_arm_z-val :lambda-list '(m))
(cl:defmethod ant_lever_arm_z-val ((m <IMUSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:ant_lever_arm_z-val is deprecated.  Use septentrio_gnss_driver-msg:ant_lever_arm_z instead.")
  (ant_lever_arm_z m))

(cl:ensure-generic-function 'theta_x-val :lambda-list '(m))
(cl:defmethod theta_x-val ((m <IMUSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:theta_x-val is deprecated.  Use septentrio_gnss_driver-msg:theta_x instead.")
  (theta_x m))

(cl:ensure-generic-function 'theta_y-val :lambda-list '(m))
(cl:defmethod theta_y-val ((m <IMUSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:theta_y-val is deprecated.  Use septentrio_gnss_driver-msg:theta_y instead.")
  (theta_y m))

(cl:ensure-generic-function 'theta_z-val :lambda-list '(m))
(cl:defmethod theta_z-val ((m <IMUSetup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:theta_z-val is deprecated.  Use septentrio_gnss_driver-msg:theta_z instead.")
  (theta_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IMUSetup>) ostream)
  "Serializes a message object of type '<IMUSetup>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'serial_port)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ant_lever_arm_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ant_lever_arm_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ant_lever_arm_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IMUSetup>) istream)
  "Deserializes a message object of type '<IMUSetup>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'serial_port)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ant_lever_arm_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ant_lever_arm_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ant_lever_arm_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IMUSetup>)))
  "Returns string type for a message object of type '<IMUSetup>"
  "septentrio_gnss_driver/IMUSetup")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IMUSetup)))
  "Returns string type for a message object of type 'IMUSetup"
  "septentrio_gnss_driver/IMUSetup")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IMUSetup>)))
  "Returns md5sum for a message object of type '<IMUSetup>"
  "da16656e2ba26ce4129ef1261f4a5033")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IMUSetup)))
  "Returns md5sum for a message object of type 'IMUSetup"
  "da16656e2ba26ce4129ef1261f4a5033")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IMUSetup>)))
  "Returns full string definition for message of type '<IMUSetup>"
  (cl:format cl:nil "# IMUSetup block~%# Block_Number 4224~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header ~%~%uint8   serial_port~%float32 ant_lever_arm_x # m~%float32 ant_lever_arm_y # m~%float32 ant_lever_arm_z # m~%float32 theta_x # deg~%float32 theta_y # deg~%float32 theta_z # deg~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IMUSetup)))
  "Returns full string definition for message of type 'IMUSetup"
  (cl:format cl:nil "# IMUSetup block~%# Block_Number 4224~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header ~%~%uint8   serial_port~%float32 ant_lever_arm_x # m~%float32 ant_lever_arm_y # m~%float32 ant_lever_arm_z # m~%float32 theta_x # deg~%float32 theta_y # deg~%float32 theta_z # deg~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IMUSetup>))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IMUSetup>))
  "Converts a ROS message object to a list"
  (cl:list 'IMUSetup
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':serial_port (serial_port msg))
    (cl:cons ':ant_lever_arm_x (ant_lever_arm_x msg))
    (cl:cons ':ant_lever_arm_y (ant_lever_arm_y msg))
    (cl:cons ':ant_lever_arm_z (ant_lever_arm_z msg))
    (cl:cons ':theta_x (theta_x msg))
    (cl:cons ':theta_y (theta_y msg))
    (cl:cons ':theta_z (theta_z msg))
))
