; Auto-generated. Do not edit!


(cl:in-package depthai_ros_msgs-msg)


;//! \htmlinclude TrackedFeatures.msg.html

(cl:defclass <TrackedFeatures> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (features
    :reader features
    :initarg :features
    :type (cl:vector depthai_ros_msgs-msg:TrackedFeature)
   :initform (cl:make-array 0 :element-type 'depthai_ros_msgs-msg:TrackedFeature :initial-element (cl:make-instance 'depthai_ros_msgs-msg:TrackedFeature))))
)

(cl:defclass TrackedFeatures (<TrackedFeatures>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackedFeatures>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackedFeatures)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name depthai_ros_msgs-msg:<TrackedFeatures> is deprecated: use depthai_ros_msgs-msg:TrackedFeatures instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrackedFeatures>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:header-val is deprecated.  Use depthai_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'features-val :lambda-list '(m))
(cl:defmethod features-val ((m <TrackedFeatures>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:features-val is deprecated.  Use depthai_ros_msgs-msg:features instead.")
  (features m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackedFeatures>) ostream)
  "Serializes a message object of type '<TrackedFeatures>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'features))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'features))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackedFeatures>) istream)
  "Deserializes a message object of type '<TrackedFeatures>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'features) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'features)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'depthai_ros_msgs-msg:TrackedFeature))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackedFeatures>)))
  "Returns string type for a message object of type '<TrackedFeatures>"
  "depthai_ros_msgs/TrackedFeatures")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackedFeatures)))
  "Returns string type for a message object of type 'TrackedFeatures"
  "depthai_ros_msgs/TrackedFeatures")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackedFeatures>)))
  "Returns md5sum for a message object of type '<TrackedFeatures>"
  "b205be15f3779117f010ff7ec03c8749")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackedFeatures)))
  "Returns md5sum for a message object of type 'TrackedFeatures"
  "b205be15f3779117f010ff7ec03c8749")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackedFeatures>)))
  "Returns full string definition for message of type '<TrackedFeatures>"
  (cl:format cl:nil "std_msgs/Header header~%depthai_ros_msgs/TrackedFeature[] features~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: depthai_ros_msgs/TrackedFeature~%std_msgs/Header header~%~%geometry_msgs/Point position~%uint32 id~%~%uint32 age~%~%float32 harris_score~%~%float32 tracking_error~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackedFeatures)))
  "Returns full string definition for message of type 'TrackedFeatures"
  (cl:format cl:nil "std_msgs/Header header~%depthai_ros_msgs/TrackedFeature[] features~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: depthai_ros_msgs/TrackedFeature~%std_msgs/Header header~%~%geometry_msgs/Point position~%uint32 id~%~%uint32 age~%~%float32 harris_score~%~%float32 tracking_error~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackedFeatures>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'features) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackedFeatures>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackedFeatures
    (cl:cons ':header (header msg))
    (cl:cons ':features (features msg))
))
