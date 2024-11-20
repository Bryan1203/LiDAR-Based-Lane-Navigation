; Auto-generated. Do not edit!


(cl:in-package depthai_ros_msgs-msg)


;//! \htmlinclude SpatialDetection.msg.html

(cl:defclass <SpatialDetection> (roslisp-msg-protocol:ros-message)
  ((results
    :reader results
    :initarg :results
    :type (cl:vector vision_msgs-msg:ObjectHypothesis)
   :initform (cl:make-array 0 :element-type 'vision_msgs-msg:ObjectHypothesis :initial-element (cl:make-instance 'vision_msgs-msg:ObjectHypothesis)))
   (bbox
    :reader bbox
    :initarg :bbox
    :type vision_msgs-msg:BoundingBox2D
    :initform (cl:make-instance 'vision_msgs-msg:BoundingBox2D))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (is_tracking
    :reader is_tracking
    :initarg :is_tracking
    :type cl:boolean
    :initform cl:nil)
   (tracking_id
    :reader tracking_id
    :initarg :tracking_id
    :type cl:string
    :initform ""))
)

(cl:defclass SpatialDetection (<SpatialDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpatialDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpatialDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name depthai_ros_msgs-msg:<SpatialDetection> is deprecated: use depthai_ros_msgs-msg:SpatialDetection instead.")))

(cl:ensure-generic-function 'results-val :lambda-list '(m))
(cl:defmethod results-val ((m <SpatialDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:results-val is deprecated.  Use depthai_ros_msgs-msg:results instead.")
  (results m))

(cl:ensure-generic-function 'bbox-val :lambda-list '(m))
(cl:defmethod bbox-val ((m <SpatialDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:bbox-val is deprecated.  Use depthai_ros_msgs-msg:bbox instead.")
  (bbox m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <SpatialDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:position-val is deprecated.  Use depthai_ros_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'is_tracking-val :lambda-list '(m))
(cl:defmethod is_tracking-val ((m <SpatialDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:is_tracking-val is deprecated.  Use depthai_ros_msgs-msg:is_tracking instead.")
  (is_tracking m))

(cl:ensure-generic-function 'tracking_id-val :lambda-list '(m))
(cl:defmethod tracking_id-val ((m <SpatialDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:tracking_id-val is deprecated.  Use depthai_ros_msgs-msg:tracking_id instead.")
  (tracking_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpatialDetection>) ostream)
  "Serializes a message object of type '<SpatialDetection>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'results))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'results))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bbox) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_tracking) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tracking_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tracking_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpatialDetection>) istream)
  "Deserializes a message object of type '<SpatialDetection>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'results) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'results)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'vision_msgs-msg:ObjectHypothesis))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bbox) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:setf (cl:slot-value msg 'is_tracking) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tracking_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tracking_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpatialDetection>)))
  "Returns string type for a message object of type '<SpatialDetection>"
  "depthai_ros_msgs/SpatialDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpatialDetection)))
  "Returns string type for a message object of type 'SpatialDetection"
  "depthai_ros_msgs/SpatialDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpatialDetection>)))
  "Returns md5sum for a message object of type '<SpatialDetection>"
  "503c7980b555f0fd79e92d14cb9ac446")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpatialDetection)))
  "Returns md5sum for a message object of type 'SpatialDetection"
  "503c7980b555f0fd79e92d14cb9ac446")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpatialDetection>)))
  "Returns full string definition for message of type '<SpatialDetection>"
  (cl:format cl:nil "~%# Class probabilities~%vision_msgs/ObjectHypothesis[] results~%~%# 2D bounding box surrounding the object.~%vision_msgs/BoundingBox2D bbox~%~%# Center of the detected object in meters ~%geometry_msgs/Point position~%~%# If true, this message contains object tracking information.~%bool is_tracking~%~%# ID used for consistency across multiple detection messages. This value will~%# likely differ from the id field set in each individual ObjectHypothesis.~%# If you set this field, be sure to also set is_tracking to True.~%string tracking_id~%================================================================================~%MSG: vision_msgs/ObjectHypothesis~%# An object hypothesis that contains no position information.~%~%# The unique numeric ID of object detected. To get additional information about~%#   this ID, such as its human-readable name, listeners should perform a lookup~%#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.~%int64 id~%~%# The probability or confidence value of the detected object. By convention,~%#   this value should lie in the range [0-1].~%float64 score~%================================================================================~%MSG: vision_msgs/BoundingBox2D~%# A 2D bounding box that can be rotated about its center.~%# All dimensions are in pixels, but represented using floating-point~%#   values to allow sub-pixel precision. If an exact pixel crop is required~%#   for a rotated bounding box, it can be calculated using Bresenham's line~%#   algorithm.~%~%# The 2D position (in pixels) and orientation of the bounding box center.~%geometry_msgs/Pose2D center~%~%# The size (in pixels) of the bounding box surrounding the object relative~%#   to the pose of its center.~%float64 size_x~%float64 size_y~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpatialDetection)))
  "Returns full string definition for message of type 'SpatialDetection"
  (cl:format cl:nil "~%# Class probabilities~%vision_msgs/ObjectHypothesis[] results~%~%# 2D bounding box surrounding the object.~%vision_msgs/BoundingBox2D bbox~%~%# Center of the detected object in meters ~%geometry_msgs/Point position~%~%# If true, this message contains object tracking information.~%bool is_tracking~%~%# ID used for consistency across multiple detection messages. This value will~%# likely differ from the id field set in each individual ObjectHypothesis.~%# If you set this field, be sure to also set is_tracking to True.~%string tracking_id~%================================================================================~%MSG: vision_msgs/ObjectHypothesis~%# An object hypothesis that contains no position information.~%~%# The unique numeric ID of object detected. To get additional information about~%#   this ID, such as its human-readable name, listeners should perform a lookup~%#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.~%int64 id~%~%# The probability or confidence value of the detected object. By convention,~%#   this value should lie in the range [0-1].~%float64 score~%================================================================================~%MSG: vision_msgs/BoundingBox2D~%# A 2D bounding box that can be rotated about its center.~%# All dimensions are in pixels, but represented using floating-point~%#   values to allow sub-pixel precision. If an exact pixel crop is required~%#   for a rotated bounding box, it can be calculated using Bresenham's line~%#   algorithm.~%~%# The 2D position (in pixels) and orientation of the bounding box center.~%geometry_msgs/Pose2D center~%~%# The size (in pixels) of the bounding box surrounding the object relative~%#   to the pose of its center.~%float64 size_x~%float64 size_y~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpatialDetection>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'results) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bbox))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     1
     4 (cl:length (cl:slot-value msg 'tracking_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpatialDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'SpatialDetection
    (cl:cons ':results (results msg))
    (cl:cons ':bbox (bbox msg))
    (cl:cons ':position (position msg))
    (cl:cons ':is_tracking (is_tracking msg))
    (cl:cons ':tracking_id (tracking_id msg))
))
