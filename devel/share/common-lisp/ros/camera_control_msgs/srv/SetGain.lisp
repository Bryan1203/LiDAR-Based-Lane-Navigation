; Auto-generated. Do not edit!


(cl:in-package camera_control_msgs-srv)


;//! \htmlinclude SetGain-request.msg.html

(cl:defclass <SetGain-request> (roslisp-msg-protocol:ros-message)
  ((target_gain
    :reader target_gain
    :initarg :target_gain
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetGain-request (<SetGain-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGain-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGain-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetGain-request> is deprecated: use camera_control_msgs-srv:SetGain-request instead.")))

(cl:ensure-generic-function 'target_gain-val :lambda-list '(m))
(cl:defmethod target_gain-val ((m <SetGain-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:target_gain-val is deprecated.  Use camera_control_msgs-srv:target_gain instead.")
  (target_gain m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGain-request>) ostream)
  "Serializes a message object of type '<SetGain-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGain-request>) istream)
  "Deserializes a message object of type '<SetGain-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_gain) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGain-request>)))
  "Returns string type for a service object of type '<SetGain-request>"
  "camera_control_msgs/SetGainRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGain-request)))
  "Returns string type for a service object of type 'SetGain-request"
  "camera_control_msgs/SetGainRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGain-request>)))
  "Returns md5sum for a message object of type '<SetGain-request>"
  "20482afb1d808bae25db22114b0a747f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGain-request)))
  "Returns md5sum for a message object of type 'SetGain-request"
  "20482afb1d808bae25db22114b0a747f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGain-request>)))
  "Returns full string definition for message of type '<SetGain-request>"
  (cl:format cl:nil "# The target gain in percent of the maximal value the camera supports.~%# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so~%# called 'device specific units'.~%float32 target_gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGain-request)))
  "Returns full string definition for message of type 'SetGain-request"
  (cl:format cl:nil "# The target gain in percent of the maximal value the camera supports.~%# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so~%# called 'device specific units'.~%float32 target_gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGain-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGain-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGain-request
    (cl:cons ':target_gain (target_gain msg))
))
;//! \htmlinclude SetGain-response.msg.html

(cl:defclass <SetGain-response> (roslisp-msg-protocol:ros-message)
  ((reached_gain
    :reader reached_gain
    :initarg :reached_gain
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetGain-response (<SetGain-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGain-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGain-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetGain-response> is deprecated: use camera_control_msgs-srv:SetGain-response instead.")))

(cl:ensure-generic-function 'reached_gain-val :lambda-list '(m))
(cl:defmethod reached_gain-val ((m <SetGain-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:reached_gain-val is deprecated.  Use camera_control_msgs-srv:reached_gain instead.")
  (reached_gain m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetGain-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:success-val is deprecated.  Use camera_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGain-response>) ostream)
  "Serializes a message object of type '<SetGain-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'reached_gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGain-response>) istream)
  "Deserializes a message object of type '<SetGain-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reached_gain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGain-response>)))
  "Returns string type for a service object of type '<SetGain-response>"
  "camera_control_msgs/SetGainResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGain-response)))
  "Returns string type for a service object of type 'SetGain-response"
  "camera_control_msgs/SetGainResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGain-response>)))
  "Returns md5sum for a message object of type '<SetGain-response>"
  "20482afb1d808bae25db22114b0a747f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGain-response)))
  "Returns md5sum for a message object of type 'SetGain-response"
  "20482afb1d808bae25db22114b0a747f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGain-response>)))
  "Returns full string definition for message of type '<SetGain-response>"
  (cl:format cl:nil "# Exact match can not always be reached~%float32 reached_gain~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGain-response)))
  "Returns full string definition for message of type 'SetGain-response"
  (cl:format cl:nil "# Exact match can not always be reached~%float32 reached_gain~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGain-response>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGain-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGain-response
    (cl:cons ':reached_gain (reached_gain msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetGain)))
  'SetGain-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetGain)))
  'SetGain-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGain)))
  "Returns string type for a service object of type '<SetGain>"
  "camera_control_msgs/SetGain")