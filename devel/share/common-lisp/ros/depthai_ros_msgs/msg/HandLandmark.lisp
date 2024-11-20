; Auto-generated. Do not edit!


(cl:in-package depthai_ros_msgs-msg)


;//! \htmlinclude HandLandmark.msg.html

(cl:defclass <HandLandmark> (roslisp-msg-protocol:ros-message)
  ((label
    :reader label
    :initarg :label
    :type cl:string
    :initform "")
   (lm_score
    :reader lm_score
    :initarg :lm_score
    :type cl:float
    :initform 0.0)
   (landmark
    :reader landmark
    :initarg :landmark
    :type (cl:vector geometry_msgs-msg:Pose2D)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose2D :initial-element (cl:make-instance 'geometry_msgs-msg:Pose2D)))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (is_spatial
    :reader is_spatial
    :initarg :is_spatial
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass HandLandmark (<HandLandmark>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HandLandmark>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HandLandmark)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name depthai_ros_msgs-msg:<HandLandmark> is deprecated: use depthai_ros_msgs-msg:HandLandmark instead.")))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <HandLandmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:label-val is deprecated.  Use depthai_ros_msgs-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'lm_score-val :lambda-list '(m))
(cl:defmethod lm_score-val ((m <HandLandmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:lm_score-val is deprecated.  Use depthai_ros_msgs-msg:lm_score instead.")
  (lm_score m))

(cl:ensure-generic-function 'landmark-val :lambda-list '(m))
(cl:defmethod landmark-val ((m <HandLandmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:landmark-val is deprecated.  Use depthai_ros_msgs-msg:landmark instead.")
  (landmark m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <HandLandmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:position-val is deprecated.  Use depthai_ros_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'is_spatial-val :lambda-list '(m))
(cl:defmethod is_spatial-val ((m <HandLandmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:is_spatial-val is deprecated.  Use depthai_ros_msgs-msg:is_spatial instead.")
  (is_spatial m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HandLandmark>) ostream)
  "Serializes a message object of type '<HandLandmark>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'label))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lm_score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'landmark))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'landmark))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_spatial) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HandLandmark>) istream)
  "Deserializes a message object of type '<HandLandmark>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lm_score) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'landmark) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'landmark)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:setf (cl:slot-value msg 'is_spatial) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HandLandmark>)))
  "Returns string type for a message object of type '<HandLandmark>"
  "depthai_ros_msgs/HandLandmark")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HandLandmark)))
  "Returns string type for a message object of type 'HandLandmark"
  "depthai_ros_msgs/HandLandmark")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HandLandmark>)))
  "Returns md5sum for a message object of type '<HandLandmark>"
  "e126f09ad65617304665e31f1a5736de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HandLandmark)))
  "Returns md5sum for a message object of type 'HandLandmark"
  "e126f09ad65617304665e31f1a5736de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HandLandmark>)))
  "Returns full string definition for message of type '<HandLandmark>"
  (cl:format cl:nil "~%# Center of the ~%~%string label~%~%# Landmarks score.~%float32 lm_score~%~%geometry_msgs/Pose2D[] landmark~%~%geometry_msgs/Point position~%bool is_spatial~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HandLandmark)))
  "Returns full string definition for message of type 'HandLandmark"
  (cl:format cl:nil "~%# Center of the ~%~%string label~%~%# Landmarks score.~%float32 lm_score~%~%geometry_msgs/Pose2D[] landmark~%~%geometry_msgs/Point position~%bool is_spatial~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HandLandmark>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'label))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'landmark) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HandLandmark>))
  "Converts a ROS message object to a list"
  (cl:list 'HandLandmark
    (cl:cons ':label (label msg))
    (cl:cons ':lm_score (lm_score msg))
    (cl:cons ':landmark (landmark msg))
    (cl:cons ':position (position msg))
    (cl:cons ':is_spatial (is_spatial msg))
))
