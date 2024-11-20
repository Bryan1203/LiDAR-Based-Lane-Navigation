; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude RFStatus.msg.html

(cl:defclass <RFStatus> (roslisp-msg-protocol:ros-message)
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
   (n
    :reader n
    :initarg :n
    :type cl:fixnum
    :initform 0)
   (sb_length
    :reader sb_length
    :initarg :sb_length
    :type cl:fixnum
    :initform 0)
   (flags
    :reader flags
    :initarg :flags
    :type cl:fixnum
    :initform 0)
   (rfband
    :reader rfband
    :initarg :rfband
    :type (cl:vector septentrio_gnss_driver-msg:RFBand)
   :initform (cl:make-array 0 :element-type 'septentrio_gnss_driver-msg:RFBand :initial-element (cl:make-instance 'septentrio_gnss_driver-msg:RFBand))))
)

(cl:defclass RFStatus (<RFStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RFStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RFStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<RFStatus> is deprecated: use septentrio_gnss_driver-msg:RFStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RFStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <RFStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <RFStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:n-val is deprecated.  Use septentrio_gnss_driver-msg:n instead.")
  (n m))

(cl:ensure-generic-function 'sb_length-val :lambda-list '(m))
(cl:defmethod sb_length-val ((m <RFStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:sb_length-val is deprecated.  Use septentrio_gnss_driver-msg:sb_length instead.")
  (sb_length m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <RFStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:flags-val is deprecated.  Use septentrio_gnss_driver-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'rfband-val :lambda-list '(m))
(cl:defmethod rfband-val ((m <RFStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:rfband-val is deprecated.  Use septentrio_gnss_driver-msg:rfband instead.")
  (rfband m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RFStatus>) ostream)
  "Serializes a message object of type '<RFStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sb_length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rfband))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'rfband))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RFStatus>) istream)
  "Deserializes a message object of type '<RFStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sb_length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rfband) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rfband)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'septentrio_gnss_driver-msg:RFBand))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RFStatus>)))
  "Returns string type for a message object of type '<RFStatus>"
  "septentrio_gnss_driver/RFStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RFStatus)))
  "Returns string type for a message object of type 'RFStatus"
  "septentrio_gnss_driver/RFStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RFStatus>)))
  "Returns md5sum for a message object of type '<RFStatus>"
  "6223e703baf4d3da19e391be72a94d34")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RFStatus)))
  "Returns md5sum for a message object of type 'RFStatus"
  "6223e703baf4d3da19e391be72a94d34")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RFStatus>)))
  "Returns full string definition for message of type '<RFStatus>"
  (cl:format cl:nil "# RFStatus block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header~%~%uint8    n~%uint8    sb_length~%uint8    flags~%RFBand[] rfband~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%================================================================================~%MSG: septentrio_gnss_driver/RFBand~%# RFband block~%~%uint32 frequency # Hz~%uint16 bandwidth # kHz~%uint8  info~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RFStatus)))
  "Returns full string definition for message of type 'RFStatus"
  (cl:format cl:nil "# RFStatus block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header~%~%uint8    n~%uint8    sb_length~%uint8    flags~%RFBand[] rfband~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%================================================================================~%MSG: septentrio_gnss_driver/RFBand~%# RFband block~%~%uint32 frequency # Hz~%uint16 bandwidth # kHz~%uint8  info~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RFStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'block_header))
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rfband) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RFStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'RFStatus
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':n (n msg))
    (cl:cons ':sb_length (sb_length msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':rfband (rfband msg))
))
