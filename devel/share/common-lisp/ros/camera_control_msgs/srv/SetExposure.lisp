; Auto-generated. Do not edit!


(cl:in-package camera_control_msgs-srv)


;//! \htmlinclude SetExposure-request.msg.html

(cl:defclass <SetExposure-request> (roslisp-msg-protocol:ros-message)
  ((target_exposure
    :reader target_exposure
    :initarg :target_exposure
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetExposure-request (<SetExposure-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetExposure-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetExposure-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetExposure-request> is deprecated: use camera_control_msgs-srv:SetExposure-request instead.")))

(cl:ensure-generic-function 'target_exposure-val :lambda-list '(m))
(cl:defmethod target_exposure-val ((m <SetExposure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:target_exposure-val is deprecated.  Use camera_control_msgs-srv:target_exposure instead.")
  (target_exposure m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetExposure-request>) ostream)
  "Serializes a message object of type '<SetExposure-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_exposure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetExposure-request>) istream)
  "Deserializes a message object of type '<SetExposure-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_exposure) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetExposure-request>)))
  "Returns string type for a service object of type '<SetExposure-request>"
  "camera_control_msgs/SetExposureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExposure-request)))
  "Returns string type for a service object of type 'SetExposure-request"
  "camera_control_msgs/SetExposureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetExposure-request>)))
  "Returns md5sum for a message object of type '<SetExposure-request>"
  "07036591479e56cbaa8b13d231a2c756")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetExposure-request)))
  "Returns md5sum for a message object of type 'SetExposure-request"
  "07036591479e56cbaa8b13d231a2c756")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetExposure-request>)))
  "Returns full string definition for message of type '<SetExposure-request>"
  (cl:format cl:nil "# The target exposure time measured in microseconds.~%# If the limits were exceeded, the desired exposure time will be truncated.~%float32 target_exposure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetExposure-request)))
  "Returns full string definition for message of type 'SetExposure-request"
  (cl:format cl:nil "# The target exposure time measured in microseconds.~%# If the limits were exceeded, the desired exposure time will be truncated.~%float32 target_exposure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetExposure-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetExposure-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetExposure-request
    (cl:cons ':target_exposure (target_exposure msg))
))
;//! \htmlinclude SetExposure-response.msg.html

(cl:defclass <SetExposure-response> (roslisp-msg-protocol:ros-message)
  ((reached_exposure
    :reader reached_exposure
    :initarg :reached_exposure
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetExposure-response (<SetExposure-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetExposure-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetExposure-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetExposure-response> is deprecated: use camera_control_msgs-srv:SetExposure-response instead.")))

(cl:ensure-generic-function 'reached_exposure-val :lambda-list '(m))
(cl:defmethod reached_exposure-val ((m <SetExposure-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:reached_exposure-val is deprecated.  Use camera_control_msgs-srv:reached_exposure instead.")
  (reached_exposure m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetExposure-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:success-val is deprecated.  Use camera_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetExposure-response>) ostream)
  "Serializes a message object of type '<SetExposure-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'reached_exposure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetExposure-response>) istream)
  "Deserializes a message object of type '<SetExposure-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reached_exposure) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetExposure-response>)))
  "Returns string type for a service object of type '<SetExposure-response>"
  "camera_control_msgs/SetExposureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExposure-response)))
  "Returns string type for a service object of type 'SetExposure-response"
  "camera_control_msgs/SetExposureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetExposure-response>)))
  "Returns md5sum for a message object of type '<SetExposure-response>"
  "07036591479e56cbaa8b13d231a2c756")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetExposure-response)))
  "Returns md5sum for a message object of type 'SetExposure-response"
  "07036591479e56cbaa8b13d231a2c756")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetExposure-response>)))
  "Returns full string definition for message of type '<SetExposure-response>"
  (cl:format cl:nil "# Exact match can not always be reached~%float32 reached_exposure~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetExposure-response)))
  "Returns full string definition for message of type 'SetExposure-response"
  (cl:format cl:nil "# Exact match can not always be reached~%float32 reached_exposure~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetExposure-response>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetExposure-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetExposure-response
    (cl:cons ':reached_exposure (reached_exposure msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetExposure)))
  'SetExposure-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetExposure)))
  'SetExposure-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetExposure)))
  "Returns string type for a service object of type '<SetExposure>"
  "camera_control_msgs/SetExposure")