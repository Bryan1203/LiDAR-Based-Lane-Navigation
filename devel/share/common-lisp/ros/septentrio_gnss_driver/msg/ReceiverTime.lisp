; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude ReceiverTime.msg.html

(cl:defclass <ReceiverTime> (roslisp-msg-protocol:ros-message)
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
   (utc_year
    :reader utc_year
    :initarg :utc_year
    :type cl:fixnum
    :initform 0)
   (utc_month
    :reader utc_month
    :initarg :utc_month
    :type cl:fixnum
    :initform 0)
   (utc_day
    :reader utc_day
    :initarg :utc_day
    :type cl:fixnum
    :initform 0)
   (utc_hour
    :reader utc_hour
    :initarg :utc_hour
    :type cl:fixnum
    :initform 0)
   (utc_min
    :reader utc_min
    :initarg :utc_min
    :type cl:fixnum
    :initform 0)
   (utc_second
    :reader utc_second
    :initarg :utc_second
    :type cl:fixnum
    :initform 0)
   (delta_ls
    :reader delta_ls
    :initarg :delta_ls
    :type cl:fixnum
    :initform 0)
   (sync_level
    :reader sync_level
    :initarg :sync_level
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ReceiverTime (<ReceiverTime>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReceiverTime>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReceiverTime)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<ReceiverTime> is deprecated: use septentrio_gnss_driver-msg:ReceiverTime instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ReceiverTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <ReceiverTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'utc_year-val :lambda-list '(m))
(cl:defmethod utc_year-val ((m <ReceiverTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:utc_year-val is deprecated.  Use septentrio_gnss_driver-msg:utc_year instead.")
  (utc_year m))

(cl:ensure-generic-function 'utc_month-val :lambda-list '(m))
(cl:defmethod utc_month-val ((m <ReceiverTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:utc_month-val is deprecated.  Use septentrio_gnss_driver-msg:utc_month instead.")
  (utc_month m))

(cl:ensure-generic-function 'utc_day-val :lambda-list '(m))
(cl:defmethod utc_day-val ((m <ReceiverTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:utc_day-val is deprecated.  Use septentrio_gnss_driver-msg:utc_day instead.")
  (utc_day m))

(cl:ensure-generic-function 'utc_hour-val :lambda-list '(m))
(cl:defmethod utc_hour-val ((m <ReceiverTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:utc_hour-val is deprecated.  Use septentrio_gnss_driver-msg:utc_hour instead.")
  (utc_hour m))

(cl:ensure-generic-function 'utc_min-val :lambda-list '(m))
(cl:defmethod utc_min-val ((m <ReceiverTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:utc_min-val is deprecated.  Use septentrio_gnss_driver-msg:utc_min instead.")
  (utc_min m))

(cl:ensure-generic-function 'utc_second-val :lambda-list '(m))
(cl:defmethod utc_second-val ((m <ReceiverTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:utc_second-val is deprecated.  Use septentrio_gnss_driver-msg:utc_second instead.")
  (utc_second m))

(cl:ensure-generic-function 'delta_ls-val :lambda-list '(m))
(cl:defmethod delta_ls-val ((m <ReceiverTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_ls-val is deprecated.  Use septentrio_gnss_driver-msg:delta_ls instead.")
  (delta_ls m))

(cl:ensure-generic-function 'sync_level-val :lambda-list '(m))
(cl:defmethod sync_level-val ((m <ReceiverTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:sync_level-val is deprecated.  Use septentrio_gnss_driver-msg:sync_level instead.")
  (sync_level m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReceiverTime>) ostream)
  "Serializes a message object of type '<ReceiverTime>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'utc_year)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'utc_month)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'utc_day)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'utc_hour)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'utc_min)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'utc_second)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'delta_ls)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sync_level)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReceiverTime>) istream)
  "Deserializes a message object of type '<ReceiverTime>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'utc_year) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'utc_month) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'utc_day) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'utc_hour) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'utc_min) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'utc_second) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'delta_ls) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sync_level)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReceiverTime>)))
  "Returns string type for a message object of type '<ReceiverTime>"
  "septentrio_gnss_driver/ReceiverTime")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReceiverTime)))
  "Returns string type for a message object of type 'ReceiverTime"
  "septentrio_gnss_driver/ReceiverTime")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReceiverTime>)))
  "Returns md5sum for a message object of type '<ReceiverTime>"
  "4e8efd4564fa50f687df91c6000b6c23")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReceiverTime)))
  "Returns md5sum for a message object of type 'ReceiverTime"
  "4e8efd4564fa50f687df91c6000b6c23")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReceiverTime>)))
  "Returns full string definition for message of type '<ReceiverTime>"
  (cl:format cl:nil "# ReceiverTime block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header~%~%int8  utc_year    # year~%int8  utc_month   # month~%int8  utc_day     # day~%int8  utc_hour    # hour~%int8  utc_min     # minute~%int8  utc_second # s~%int8  delta_ls   # s~%uint8 sync_level~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReceiverTime)))
  "Returns full string definition for message of type 'ReceiverTime"
  (cl:format cl:nil "# ReceiverTime block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header~%~%int8  utc_year    # year~%int8  utc_month   # month~%int8  utc_day     # day~%int8  utc_hour    # hour~%int8  utc_min     # minute~%int8  utc_second # s~%int8  delta_ls   # s~%uint8 sync_level~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReceiverTime>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'block_header))
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReceiverTime>))
  "Converts a ROS message object to a list"
  (cl:list 'ReceiverTime
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':utc_year (utc_year msg))
    (cl:cons ':utc_month (utc_month msg))
    (cl:cons ':utc_day (utc_day msg))
    (cl:cons ':utc_hour (utc_hour msg))
    (cl:cons ':utc_min (utc_min msg))
    (cl:cons ':utc_second (utc_second msg))
    (cl:cons ':delta_ls (delta_ls msg))
    (cl:cons ':sync_level (sync_level msg))
))
