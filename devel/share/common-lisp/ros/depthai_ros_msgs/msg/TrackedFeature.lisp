; Auto-generated. Do not edit!


(cl:in-package depthai_ros_msgs-msg)


;//! \htmlinclude TrackedFeature.msg.html

(cl:defclass <TrackedFeature> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (age
    :reader age
    :initarg :age
    :type cl:integer
    :initform 0)
   (harris_score
    :reader harris_score
    :initarg :harris_score
    :type cl:float
    :initform 0.0)
   (tracking_error
    :reader tracking_error
    :initarg :tracking_error
    :type cl:float
    :initform 0.0))
)

(cl:defclass TrackedFeature (<TrackedFeature>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedFeature>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedFeature)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name depthai_ros_msgs-msg:<TrackedFeature> is deprecated: use depthai_ros_msgs-msg:TrackedFeature instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrackedFeature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:header-val is deprecated.  Use depthai_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <TrackedFeature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:position-val is deprecated.  Use depthai_ros_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <TrackedFeature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:id-val is deprecated.  Use depthai_ros_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'age-val :lambda-list '(m))
(cl:defmethod age-val ((m <TrackedFeature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:age-val is deprecated.  Use depthai_ros_msgs-msg:age instead.")
  (age m))

(cl:ensure-generic-function 'harris_score-val :lambda-list '(m))
(cl:defmethod harris_score-val ((m <TrackedFeature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:harris_score-val is deprecated.  Use depthai_ros_msgs-msg:harris_score instead.")
  (harris_score m))

(cl:ensure-generic-function 'tracking_error-val :lambda-list '(m))
(cl:defmethod tracking_error-val ((m <TrackedFeature>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:tracking_error-val is deprecated.  Use depthai_ros_msgs-msg:tracking_error instead.")
  (tracking_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedFeature>) ostream)
  "Serializes a message object of type '<TrackedFeature>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'age)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'age)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'age)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'age)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'harris_score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tracking_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedFeature>) istream)
  "Deserializes a message object of type '<TrackedFeature>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'age)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'age)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'age)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'age)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'harris_score) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tracking_error) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedFeature>)))
  "Returns string type for a message object of type '<TrackedFeature>"
  "depthai_ros_msgs/TrackedFeature")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedFeature)))
  "Returns string type for a message object of type 'TrackedFeature"
  "depthai_ros_msgs/TrackedFeature")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedFeature>)))
  "Returns md5sum for a message object of type '<TrackedFeature>"
  "7b24a14f9b5d900250ec5c3f7bf1ec5e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedFeature)))
  "Returns md5sum for a message object of type 'TrackedFeature"
  "7b24a14f9b5d900250ec5c3f7bf1ec5e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedFeature>)))
  "Returns full string definition for message of type '<TrackedFeature>"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Point position~%uint32 id~%~%uint32 age~%~%float32 harris_score~%~%float32 tracking_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedFeature)))
  "Returns full string definition for message of type 'TrackedFeature"
  (cl:format cl:nil "std_msgs/Header header~%~%geometry_msgs/Point position~%uint32 id~%~%uint32 age~%~%float32 harris_score~%~%float32 tracking_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedFeature>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedFeature>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedFeature
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':id (id msg))
    (cl:cons ':age (age msg))
    (cl:cons ':harris_score (harris_score msg))
    (cl:cons ':tracking_error (tracking_error msg))
))
