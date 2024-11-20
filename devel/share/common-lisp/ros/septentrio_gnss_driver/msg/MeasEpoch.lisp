; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude MeasEpoch.msg.html

(cl:defclass <MeasEpoch> (roslisp-msg-protocol:ros-message)
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
   (sb1_length
    :reader sb1_length
    :initarg :sb1_length
    :type cl:fixnum
    :initform 0)
   (sb2_length
    :reader sb2_length
    :initarg :sb2_length
    :type cl:fixnum
    :initform 0)
   (common_flags
    :reader common_flags
    :initarg :common_flags
    :type cl:fixnum
    :initform 0)
   (cum_clk_jumps
    :reader cum_clk_jumps
    :initarg :cum_clk_jumps
    :type cl:fixnum
    :initform 0)
   (type1
    :reader type1
    :initarg :type1
    :type (cl:vector septentrio_gnss_driver-msg:MeasEpochChannelType1)
   :initform (cl:make-array 0 :element-type 'septentrio_gnss_driver-msg:MeasEpochChannelType1 :initial-element (cl:make-instance 'septentrio_gnss_driver-msg:MeasEpochChannelType1))))
)

(cl:defclass MeasEpoch (<MeasEpoch>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MeasEpoch>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MeasEpoch)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<MeasEpoch> is deprecated: use septentrio_gnss_driver-msg:MeasEpoch instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MeasEpoch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <MeasEpoch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <MeasEpoch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:n-val is deprecated.  Use septentrio_gnss_driver-msg:n instead.")
  (n m))

(cl:ensure-generic-function 'sb1_length-val :lambda-list '(m))
(cl:defmethod sb1_length-val ((m <MeasEpoch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:sb1_length-val is deprecated.  Use septentrio_gnss_driver-msg:sb1_length instead.")
  (sb1_length m))

(cl:ensure-generic-function 'sb2_length-val :lambda-list '(m))
(cl:defmethod sb2_length-val ((m <MeasEpoch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:sb2_length-val is deprecated.  Use septentrio_gnss_driver-msg:sb2_length instead.")
  (sb2_length m))

(cl:ensure-generic-function 'common_flags-val :lambda-list '(m))
(cl:defmethod common_flags-val ((m <MeasEpoch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:common_flags-val is deprecated.  Use septentrio_gnss_driver-msg:common_flags instead.")
  (common_flags m))

(cl:ensure-generic-function 'cum_clk_jumps-val :lambda-list '(m))
(cl:defmethod cum_clk_jumps-val ((m <MeasEpoch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cum_clk_jumps-val is deprecated.  Use septentrio_gnss_driver-msg:cum_clk_jumps instead.")
  (cum_clk_jumps m))

(cl:ensure-generic-function 'type1-val :lambda-list '(m))
(cl:defmethod type1-val ((m <MeasEpoch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:type1-val is deprecated.  Use septentrio_gnss_driver-msg:type1 instead.")
  (type1 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MeasEpoch>) ostream)
  "Serializes a message object of type '<MeasEpoch>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sb1_length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sb2_length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'common_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cum_clk_jumps)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'type1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'type1))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MeasEpoch>) istream)
  "Deserializes a message object of type '<MeasEpoch>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sb1_length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sb2_length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'common_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cum_clk_jumps)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'type1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'type1)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'septentrio_gnss_driver-msg:MeasEpochChannelType1))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MeasEpoch>)))
  "Returns string type for a message object of type '<MeasEpoch>"
  "septentrio_gnss_driver/MeasEpoch")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MeasEpoch)))
  "Returns string type for a message object of type 'MeasEpoch"
  "septentrio_gnss_driver/MeasEpoch")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MeasEpoch>)))
  "Returns md5sum for a message object of type '<MeasEpoch>"
  "4cf9a1d78edfe10d2a772f4a50dd0645")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MeasEpoch)))
  "Returns md5sum for a message object of type 'MeasEpoch"
  "4cf9a1d78edfe10d2a772f4a50dd0645")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MeasEpoch>)))
  "Returns full string definition for message of type '<MeasEpoch>"
  (cl:format cl:nil "# MeasEpoch block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8 n~%uint8 sb1_length~%uint8 sb2_length      ~%uint8 common_flags~%uint8 cum_clk_jumps # 0.001 s~%~%MeasEpochChannelType1[] type1~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%================================================================================~%MSG: septentrio_gnss_driver/MeasEpochChannelType1~%# MeasEpochChannelType1 block~%   ~%uint8  rx_channel~%uint8  type~%uint8  sv_id~%uint8  misc        # 4294967.296 m~%uint32 code_lsb    # 0.001 m~%int32  doppler     # 0.0001 Hz~%uint16 carrier_lsb # 0.001 cycles~%int8   carrier_msb # 65.536 cycles~%uint8  cn0         # 0.25 dB-Hz~%uint16 lock_time~%uint8  obs_info~%uint8  n2~%~%MeasEpochChannelType2[] type2~%================================================================================~%MSG: septentrio_gnss_driver/MeasEpochChannelType2~%# MeasEpochChannelType2 block~%   ~%uint8  type~%uint8  lock_time          # s~%uint8  cn0                # 0.25 dB-Hz~%uint8  offsets_msb        # 65.536 m or 65.536 Hz~%int8   carrier_msb        # 65.536 cycles~%uint8  obs_info~%uint16 code_offset_lsb    # 0.001 m~%uint16 carrier_lsb        # 0.001 cycles~%uint16 doppler_offset_lsb # 0.0001 Hz~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MeasEpoch)))
  "Returns full string definition for message of type 'MeasEpoch"
  (cl:format cl:nil "# MeasEpoch block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8 n~%uint8 sb1_length~%uint8 sb2_length      ~%uint8 common_flags~%uint8 cum_clk_jumps # 0.001 s~%~%MeasEpochChannelType1[] type1~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%================================================================================~%MSG: septentrio_gnss_driver/MeasEpochChannelType1~%# MeasEpochChannelType1 block~%   ~%uint8  rx_channel~%uint8  type~%uint8  sv_id~%uint8  misc        # 4294967.296 m~%uint32 code_lsb    # 0.001 m~%int32  doppler     # 0.0001 Hz~%uint16 carrier_lsb # 0.001 cycles~%int8   carrier_msb # 65.536 cycles~%uint8  cn0         # 0.25 dB-Hz~%uint16 lock_time~%uint8  obs_info~%uint8  n2~%~%MeasEpochChannelType2[] type2~%================================================================================~%MSG: septentrio_gnss_driver/MeasEpochChannelType2~%# MeasEpochChannelType2 block~%   ~%uint8  type~%uint8  lock_time          # s~%uint8  cn0                # 0.25 dB-Hz~%uint8  offsets_msb        # 65.536 m or 65.536 Hz~%int8   carrier_msb        # 65.536 cycles~%uint8  obs_info~%uint16 code_offset_lsb    # 0.001 m~%uint16 carrier_lsb        # 0.001 cycles~%uint16 doppler_offset_lsb # 0.0001 Hz~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MeasEpoch>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'block_header))
     1
     1
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'type1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MeasEpoch>))
  "Converts a ROS message object to a list"
  (cl:list 'MeasEpoch
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':n (n msg))
    (cl:cons ':sb1_length (sb1_length msg))
    (cl:cons ':sb2_length (sb2_length msg))
    (cl:cons ':common_flags (common_flags msg))
    (cl:cons ':cum_clk_jumps (cum_clk_jumps msg))
    (cl:cons ':type1 (type1 msg))
))
