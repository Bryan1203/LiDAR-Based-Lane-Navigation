; Auto-generated. Do not edit!


(cl:in-package camera_control_msgs-srv)


;//! \htmlinclude SetGamma-request.msg.html

(cl:defclass <SetGamma-request> (roslisp-msg-protocol:ros-message)
  ((target_gamma
    :reader target_gamma
    :initarg :target_gamma
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetGamma-request (<SetGamma-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGamma-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGamma-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetGamma-request> is deprecated: use camera_control_msgs-srv:SetGamma-request instead.")))

(cl:ensure-generic-function 'target_gamma-val :lambda-list '(m))
(cl:defmethod target_gamma-val ((m <SetGamma-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:target_gamma-val is deprecated.  Use camera_control_msgs-srv:target_gamma instead.")
  (target_gamma m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGamma-request>) ostream)
  "Serializes a message object of type '<SetGamma-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_gamma))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGamma-request>) istream)
  "Deserializes a message object of type '<SetGamma-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_gamma) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGamma-request>)))
  "Returns string type for a service object of type '<SetGamma-request>"
  "camera_control_msgs/SetGammaRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGamma-request)))
  "Returns string type for a service object of type 'SetGamma-request"
  "camera_control_msgs/SetGammaRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGamma-request>)))
  "Returns md5sum for a message object of type '<SetGamma-request>"
  "d071cb93e1f4c5737ce40debcd7942a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGamma-request)))
  "Returns md5sum for a message object of type 'SetGamma-request"
  "d071cb93e1f4c5737ce40debcd7942a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGamma-request>)))
  "Returns full string definition for message of type '<SetGamma-request>"
  (cl:format cl:nil "# The target gamma correction of pixel intensity.~%# Adjusts the brightness of the pixel values output by the camera's sensor~%# to account for a non-linearity in the human perception of brightness or~%# of the display system (such as CRT).~%float32 target_gamma~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGamma-request)))
  "Returns full string definition for message of type 'SetGamma-request"
  (cl:format cl:nil "# The target gamma correction of pixel intensity.~%# Adjusts the brightness of the pixel values output by the camera's sensor~%# to account for a non-linearity in the human perception of brightness or~%# of the display system (such as CRT).~%float32 target_gamma~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGamma-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGamma-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGamma-request
    (cl:cons ':target_gamma (target_gamma msg))
))
;//! \htmlinclude SetGamma-response.msg.html

(cl:defclass <SetGamma-response> (roslisp-msg-protocol:ros-message)
  ((reached_gamma
    :reader reached_gamma
    :initarg :reached_gamma
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetGamma-response (<SetGamma-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGamma-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGamma-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetGamma-response> is deprecated: use camera_control_msgs-srv:SetGamma-response instead.")))

(cl:ensure-generic-function 'reached_gamma-val :lambda-list '(m))
(cl:defmethod reached_gamma-val ((m <SetGamma-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:reached_gamma-val is deprecated.  Use camera_control_msgs-srv:reached_gamma instead.")
  (reached_gamma m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetGamma-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:success-val is deprecated.  Use camera_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGamma-response>) ostream)
  "Serializes a message object of type '<SetGamma-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'reached_gamma))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGamma-response>) istream)
  "Deserializes a message object of type '<SetGamma-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reached_gamma) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGamma-response>)))
  "Returns string type for a service object of type '<SetGamma-response>"
  "camera_control_msgs/SetGammaResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGamma-response)))
  "Returns string type for a service object of type 'SetGamma-response"
  "camera_control_msgs/SetGammaResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGamma-response>)))
  "Returns md5sum for a message object of type '<SetGamma-response>"
  "d071cb93e1f4c5737ce40debcd7942a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGamma-response)))
  "Returns md5sum for a message object of type 'SetGamma-response"
  "d071cb93e1f4c5737ce40debcd7942a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGamma-response>)))
  "Returns full string definition for message of type '<SetGamma-response>"
  (cl:format cl:nil "# Exact match can not always be reached~%float32 reached_gamma~%bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGamma-response)))
  "Returns full string definition for message of type 'SetGamma-response"
  (cl:format cl:nil "# Exact match can not always be reached~%float32 reached_gamma~%bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGamma-response>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGamma-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGamma-response
    (cl:cons ':reached_gamma (reached_gamma msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetGamma)))
  'SetGamma-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetGamma)))
  'SetGamma-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGamma)))
  "Returns string type for a service object of type '<SetGamma>"
  "camera_control_msgs/SetGamma")