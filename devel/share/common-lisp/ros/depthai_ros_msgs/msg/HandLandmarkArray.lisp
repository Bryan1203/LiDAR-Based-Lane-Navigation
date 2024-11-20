; Auto-generated. Do not edit!


(cl:in-package depthai_ros_msgs-msg)


;//! \htmlinclude HandLandmarkArray.msg.html

(cl:defclass <HandLandmarkArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (landmarks
    :reader landmarks
    :initarg :landmarks
    :type (cl:vector depthai_ros_msgs-msg:HandLandmark)
   :initform (cl:make-array 0 :element-type 'depthai_ros_msgs-msg:HandLandmark :initial-element (cl:make-instance 'depthai_ros_msgs-msg:HandLandmark))))
)

(cl:defclass HandLandmarkArray (<HandLandmarkArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandLandmarkArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandLandmarkArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name depthai_ros_msgs-msg:<HandLandmarkArray> is deprecated: use depthai_ros_msgs-msg:HandLandmarkArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HandLandmarkArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:header-val is deprecated.  Use depthai_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'landmarks-val :lambda-list '(m))
(cl:defmethod landmarks-val ((m <HandLandmarkArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:landmarks-val is deprecated.  Use depthai_ros_msgs-msg:landmarks instead.")
  (landmarks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandLandmarkArray>) ostream)
  "Serializes a message object of type '<HandLandmarkArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'landmarks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'landmarks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandLandmarkArray>) istream)
  "Deserializes a message object of type '<HandLandmarkArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'landmarks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'landmarks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'depthai_ros_msgs-msg:HandLandmark))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandLandmarkArray>)))
  "Returns string type for a message object of type '<HandLandmarkArray>"
  "depthai_ros_msgs/HandLandmarkArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandLandmarkArray)))
  "Returns string type for a message object of type 'HandLandmarkArray"
  "depthai_ros_msgs/HandLandmarkArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandLandmarkArray>)))
  "Returns md5sum for a message object of type '<HandLandmarkArray>"
  "9215903cb0dba0f3a32440758d1e30e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandLandmarkArray)))
  "Returns md5sum for a message object of type 'HandLandmarkArray"
  "9215903cb0dba0f3a32440758d1e30e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandLandmarkArray>)))
  "Returns full string definition for message of type '<HandLandmarkArray>"
  (cl:format cl:nil "# A list of hand landmark detections~%~%std_msgs/Header header~%# A list of the detected proposals. A multi-proposal detector might generate along with the 3D depth information~%#   this list with many candidate detections generated from a single input.~%HandLandmark[] landmarks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: depthai_ros_msgs/HandLandmark~%~%# Center of the ~%~%string label~%~%# Landmarks score.~%float32 lm_score~%~%geometry_msgs/Pose2D[] landmark~%~%geometry_msgs/Point position~%bool is_spatial~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandLandmarkArray)))
  "Returns full string definition for message of type 'HandLandmarkArray"
  (cl:format cl:nil "# A list of hand landmark detections~%~%std_msgs/Header header~%# A list of the detected proposals. A multi-proposal detector might generate along with the 3D depth information~%#   this list with many candidate detections generated from a single input.~%HandLandmark[] landmarks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: depthai_ros_msgs/HandLandmark~%~%# Center of the ~%~%string label~%~%# Landmarks score.~%float32 lm_score~%~%geometry_msgs/Pose2D[] landmark~%~%geometry_msgs/Point position~%bool is_spatial~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandLandmarkArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'landmarks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandLandmarkArray>))
  "Converts a ROS message object to a list"
  (cl:list 'HandLandmarkArray
    (cl:cons ':header (header msg))
    (cl:cons ':landmarks (landmarks msg))
))
