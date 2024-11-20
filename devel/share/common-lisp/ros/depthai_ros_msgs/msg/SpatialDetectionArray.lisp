; Auto-generated. Do not edit!


(cl:in-package depthai_ros_msgs-msg)


;//! \htmlinclude SpatialDetectionArray.msg.html

(cl:defclass <SpatialDetectionArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector depthai_ros_msgs-msg:SpatialDetection)
   :initform (cl:make-array 0 :element-type 'depthai_ros_msgs-msg:SpatialDetection :initial-element (cl:make-instance 'depthai_ros_msgs-msg:SpatialDetection))))
)

(cl:defclass SpatialDetectionArray (<SpatialDetectionArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpatialDetectionArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpatialDetectionArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name depthai_ros_msgs-msg:<SpatialDetectionArray> is deprecated: use depthai_ros_msgs-msg:SpatialDetectionArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SpatialDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:header-val is deprecated.  Use depthai_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <SpatialDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:detections-val is deprecated.  Use depthai_ros_msgs-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpatialDetectionArray>) ostream)
  "Serializes a message object of type '<SpatialDetectionArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpatialDetectionArray>) istream)
  "Deserializes a message object of type '<SpatialDetectionArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'depthai_ros_msgs-msg:SpatialDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpatialDetectionArray>)))
  "Returns string type for a message object of type '<SpatialDetectionArray>"
  "depthai_ros_msgs/SpatialDetectionArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpatialDetectionArray)))
  "Returns string type for a message object of type 'SpatialDetectionArray"
  "depthai_ros_msgs/SpatialDetectionArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpatialDetectionArray>)))
  "Returns md5sum for a message object of type '<SpatialDetectionArray>"
  "f3caf8c6374b91f3a4b93f9610044116")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpatialDetectionArray)))
  "Returns md5sum for a message object of type 'SpatialDetectionArray"
  "f3caf8c6374b91f3a4b93f9610044116")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpatialDetectionArray>)))
  "Returns full string definition for message of type '<SpatialDetectionArray>"
  (cl:format cl:nil "# A list of 2D detections, for a multi-object 2D detector along with 3D depth information.~%~%std_msgs/Header header~%# A list of the detected proposals. A multi-proposal detector might generate along with the 3D depth information~%#   this list with many candidate detections generated from a single input.~%SpatialDetection[] detections~%~%## These messages are created as close as possible to the official vision_msgs(http://wiki.ros.org/vision_msgs)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: depthai_ros_msgs/SpatialDetection~%~%# Class probabilities~%vision_msgs/ObjectHypothesis[] results~%~%# 2D bounding box surrounding the object.~%vision_msgs/BoundingBox2D bbox~%~%# Center of the detected object in meters ~%geometry_msgs/Point position~%~%# If true, this message contains object tracking information.~%bool is_tracking~%~%# ID used for consistency across multiple detection messages. This value will~%# likely differ from the id field set in each individual ObjectHypothesis.~%# If you set this field, be sure to also set is_tracking to True.~%string tracking_id~%================================================================================~%MSG: vision_msgs/ObjectHypothesis~%# An object hypothesis that contains no position information.~%~%# The unique numeric ID of object detected. To get additional information about~%#   this ID, such as its human-readable name, listeners should perform a lookup~%#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.~%int64 id~%~%# The probability or confidence value of the detected object. By convention,~%#   this value should lie in the range [0-1].~%float64 score~%================================================================================~%MSG: vision_msgs/BoundingBox2D~%# A 2D bounding box that can be rotated about its center.~%# All dimensions are in pixels, but represented using floating-point~%#   values to allow sub-pixel precision. If an exact pixel crop is required~%#   for a rotated bounding box, it can be calculated using Bresenham's line~%#   algorithm.~%~%# The 2D position (in pixels) and orientation of the bounding box center.~%geometry_msgs/Pose2D center~%~%# The size (in pixels) of the bounding box surrounding the object relative~%#   to the pose of its center.~%float64 size_x~%float64 size_y~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpatialDetectionArray)))
  "Returns full string definition for message of type 'SpatialDetectionArray"
  (cl:format cl:nil "# A list of 2D detections, for a multi-object 2D detector along with 3D depth information.~%~%std_msgs/Header header~%# A list of the detected proposals. A multi-proposal detector might generate along with the 3D depth information~%#   this list with many candidate detections generated from a single input.~%SpatialDetection[] detections~%~%## These messages are created as close as possible to the official vision_msgs(http://wiki.ros.org/vision_msgs)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: depthai_ros_msgs/SpatialDetection~%~%# Class probabilities~%vision_msgs/ObjectHypothesis[] results~%~%# 2D bounding box surrounding the object.~%vision_msgs/BoundingBox2D bbox~%~%# Center of the detected object in meters ~%geometry_msgs/Point position~%~%# If true, this message contains object tracking information.~%bool is_tracking~%~%# ID used for consistency across multiple detection messages. This value will~%# likely differ from the id field set in each individual ObjectHypothesis.~%# If you set this field, be sure to also set is_tracking to True.~%string tracking_id~%================================================================================~%MSG: vision_msgs/ObjectHypothesis~%# An object hypothesis that contains no position information.~%~%# The unique numeric ID of object detected. To get additional information about~%#   this ID, such as its human-readable name, listeners should perform a lookup~%#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.~%int64 id~%~%# The probability or confidence value of the detected object. By convention,~%#   this value should lie in the range [0-1].~%float64 score~%================================================================================~%MSG: vision_msgs/BoundingBox2D~%# A 2D bounding box that can be rotated about its center.~%# All dimensions are in pixels, but represented using floating-point~%#   values to allow sub-pixel precision. If an exact pixel crop is required~%#   for a rotated bounding box, it can be calculated using Bresenham's line~%#   algorithm.~%~%# The 2D position (in pixels) and orientation of the bounding box center.~%geometry_msgs/Pose2D center~%~%# The size (in pixels) of the bounding box surrounding the object relative~%#   to the pose of its center.~%float64 size_x~%float64 size_y~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpatialDetectionArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpatialDetectionArray>))
  "Converts a ROS message object to a list"
  (cl:list 'SpatialDetectionArray
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
))
