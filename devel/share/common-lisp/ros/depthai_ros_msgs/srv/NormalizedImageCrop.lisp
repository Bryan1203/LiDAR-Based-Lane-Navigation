; Auto-generated. Do not edit!


(cl:in-package depthai_ros_msgs-srv)


;//! \htmlinclude NormalizedImageCrop-request.msg.html

(cl:defclass <NormalizedImageCrop-request> (roslisp-msg-protocol:ros-message)
  ((top_left
    :reader top_left
    :initarg :top_left
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (bottom_right
    :reader bottom_right
    :initarg :bottom_right
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D)))
)

(cl:defclass NormalizedImageCrop-request (<NormalizedImageCrop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NormalizedImageCrop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NormalizedImageCrop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name depthai_ros_msgs-srv:<NormalizedImageCrop-request> is deprecated: use depthai_ros_msgs-srv:NormalizedImageCrop-request instead.")))

(cl:ensure-generic-function 'top_left-val :lambda-list '(m))
(cl:defmethod top_left-val ((m <NormalizedImageCrop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-srv:top_left-val is deprecated.  Use depthai_ros_msgs-srv:top_left instead.")
  (top_left m))

(cl:ensure-generic-function 'bottom_right-val :lambda-list '(m))
(cl:defmethod bottom_right-val ((m <NormalizedImageCrop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-srv:bottom_right-val is deprecated.  Use depthai_ros_msgs-srv:bottom_right instead.")
  (bottom_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NormalizedImageCrop-request>) ostream)
  "Serializes a message object of type '<NormalizedImageCrop-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'top_left) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bottom_right) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NormalizedImageCrop-request>) istream)
  "Deserializes a message object of type '<NormalizedImageCrop-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'top_left) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bottom_right) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NormalizedImageCrop-request>)))
  "Returns string type for a service object of type '<NormalizedImageCrop-request>"
  "depthai_ros_msgs/NormalizedImageCropRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NormalizedImageCrop-request)))
  "Returns string type for a service object of type 'NormalizedImageCrop-request"
  "depthai_ros_msgs/NormalizedImageCropRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NormalizedImageCrop-request>)))
  "Returns md5sum for a message object of type '<NormalizedImageCrop-request>"
  "35d4b74aa140d5d6ce7c27c43314e948")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NormalizedImageCrop-request)))
  "Returns md5sum for a message object of type 'NormalizedImageCrop-request"
  "35d4b74aa140d5d6ce7c27c43314e948")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NormalizedImageCrop-request>)))
  "Returns full string definition for message of type '<NormalizedImageCrop-request>"
  (cl:format cl:nil "geometry_msgs/Pose2D top_left~%geometry_msgs/Pose2D bottom_right~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NormalizedImageCrop-request)))
  "Returns full string definition for message of type 'NormalizedImageCrop-request"
  (cl:format cl:nil "geometry_msgs/Pose2D top_left~%geometry_msgs/Pose2D bottom_right~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NormalizedImageCrop-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'top_left))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bottom_right))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NormalizedImageCrop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'NormalizedImageCrop-request
    (cl:cons ':top_left (top_left msg))
    (cl:cons ':bottom_right (bottom_right msg))
))
;//! \htmlinclude NormalizedImageCrop-response.msg.html

(cl:defclass <NormalizedImageCrop-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:integer
    :initform 0))
)

(cl:defclass NormalizedImageCrop-response (<NormalizedImageCrop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NormalizedImageCrop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NormalizedImageCrop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name depthai_ros_msgs-srv:<NormalizedImageCrop-response> is deprecated: use depthai_ros_msgs-srv:NormalizedImageCrop-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <NormalizedImageCrop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-srv:status-val is deprecated.  Use depthai_ros_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NormalizedImageCrop-response>) ostream)
  "Serializes a message object of type '<NormalizedImageCrop-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NormalizedImageCrop-response>) istream)
  "Deserializes a message object of type '<NormalizedImageCrop-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NormalizedImageCrop-response>)))
  "Returns string type for a service object of type '<NormalizedImageCrop-response>"
  "depthai_ros_msgs/NormalizedImageCropResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NormalizedImageCrop-response)))
  "Returns string type for a service object of type 'NormalizedImageCrop-response"
  "depthai_ros_msgs/NormalizedImageCropResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NormalizedImageCrop-response>)))
  "Returns md5sum for a message object of type '<NormalizedImageCrop-response>"
  "35d4b74aa140d5d6ce7c27c43314e948")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NormalizedImageCrop-response)))
  "Returns md5sum for a message object of type 'NormalizedImageCrop-response"
  "35d4b74aa140d5d6ce7c27c43314e948")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NormalizedImageCrop-response>)))
  "Returns full string definition for message of type '<NormalizedImageCrop-response>"
  (cl:format cl:nil "int64 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NormalizedImageCrop-response)))
  "Returns full string definition for message of type 'NormalizedImageCrop-response"
  (cl:format cl:nil "int64 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NormalizedImageCrop-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NormalizedImageCrop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'NormalizedImageCrop-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'NormalizedImageCrop)))
  'NormalizedImageCrop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'NormalizedImageCrop)))
  'NormalizedImageCrop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NormalizedImageCrop)))
  "Returns string type for a service object of type '<NormalizedImageCrop>"
  "depthai_ros_msgs/NormalizedImageCrop")