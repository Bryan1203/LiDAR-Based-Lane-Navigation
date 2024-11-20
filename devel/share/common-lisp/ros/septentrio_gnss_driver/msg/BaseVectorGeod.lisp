; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude BaseVectorGeod.msg.html

(cl:defclass <BaseVectorGeod> (roslisp-msg-protocol:ros-message)
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
   (vector_info_geod
    :reader vector_info_geod
    :initarg :vector_info_geod
    :type (cl:vector septentrio_gnss_driver-msg:VectorInfoGeod)
   :initform (cl:make-array 0 :element-type 'septentrio_gnss_driver-msg:VectorInfoGeod :initial-element (cl:make-instance 'septentrio_gnss_driver-msg:VectorInfoGeod))))
)

(cl:defclass BaseVectorGeod (<BaseVectorGeod>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BaseVectorGeod>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BaseVectorGeod)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<BaseVectorGeod> is deprecated: use septentrio_gnss_driver-msg:BaseVectorGeod instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BaseVectorGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <BaseVectorGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <BaseVectorGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:n-val is deprecated.  Use septentrio_gnss_driver-msg:n instead.")
  (n m))

(cl:ensure-generic-function 'sb_length-val :lambda-list '(m))
(cl:defmethod sb_length-val ((m <BaseVectorGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:sb_length-val is deprecated.  Use septentrio_gnss_driver-msg:sb_length instead.")
  (sb_length m))

(cl:ensure-generic-function 'vector_info_geod-val :lambda-list '(m))
(cl:defmethod vector_info_geod-val ((m <BaseVectorGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vector_info_geod-val is deprecated.  Use septentrio_gnss_driver-msg:vector_info_geod instead.")
  (vector_info_geod m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BaseVectorGeod>) ostream)
  "Serializes a message object of type '<BaseVectorGeod>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sb_length)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vector_info_geod))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'vector_info_geod))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BaseVectorGeod>) istream)
  "Deserializes a message object of type '<BaseVectorGeod>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sb_length)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vector_info_geod) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vector_info_geod)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'septentrio_gnss_driver-msg:VectorInfoGeod))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BaseVectorGeod>)))
  "Returns string type for a message object of type '<BaseVectorGeod>"
  "septentrio_gnss_driver/BaseVectorGeod")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BaseVectorGeod)))
  "Returns string type for a message object of type 'BaseVectorGeod"
  "septentrio_gnss_driver/BaseVectorGeod")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BaseVectorGeod>)))
  "Returns md5sum for a message object of type '<BaseVectorGeod>"
  "1349428be748ae82d66f928c5daf71dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BaseVectorGeod)))
  "Returns md5sum for a message object of type 'BaseVectorGeod"
  "1349428be748ae82d66f928c5daf71dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BaseVectorGeod>)))
  "Returns full string definition for message of type '<BaseVectorGeod>"
  (cl:format cl:nil "# BaseVectorGeod block ~%# Block_Number 4028~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header~%~%uint8   n~%uint8   sb_length~%~%VectorInfoGeod[] vector_info_geod ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%================================================================================~%MSG: septentrio_gnss_driver/VectorInfoGeod~%# VectorInfoGeod block~%   ~%uint8   nr_sv~%uint8   error~%uint8   mode~%uint8   misc~%float64 delta_east   # m~%float64 delta_north  # m~%float64 delta_up     # m~%float32 delta_ve     # m~%float32 delta_vn     # m~%float32 delta_vu     # m~%uint16  azimuth      # 0.01 deg~%int16   elevation    # 0.01 deg~%uint16  reference_id~%uint16  corr_age     # 0.01 s~%uint32  signal_info~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BaseVectorGeod)))
  "Returns full string definition for message of type 'BaseVectorGeod"
  (cl:format cl:nil "# BaseVectorGeod block ~%# Block_Number 4028~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header~%~%uint8   n~%uint8   sb_length~%~%VectorInfoGeod[] vector_info_geod ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%================================================================================~%MSG: septentrio_gnss_driver/VectorInfoGeod~%# VectorInfoGeod block~%   ~%uint8   nr_sv~%uint8   error~%uint8   mode~%uint8   misc~%float64 delta_east   # m~%float64 delta_north  # m~%float64 delta_up     # m~%float32 delta_ve     # m~%float32 delta_vn     # m~%float32 delta_vu     # m~%uint16  azimuth      # 0.01 deg~%int16   elevation    # 0.01 deg~%uint16  reference_id~%uint16  corr_age     # 0.01 s~%uint32  signal_info~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BaseVectorGeod>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'block_header))
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vector_info_geod) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BaseVectorGeod>))
  "Converts a ROS message object to a list"
  (cl:list 'BaseVectorGeod
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':n (n msg))
    (cl:cons ':sb_length (sb_length msg))
    (cl:cons ':vector_info_geod (vector_info_geod msg))
))
