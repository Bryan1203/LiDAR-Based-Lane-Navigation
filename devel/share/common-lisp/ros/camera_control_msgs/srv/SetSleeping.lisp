; Auto-generated. Do not edit!


(cl:in-package camera_control_msgs-srv)


;//! \htmlinclude SetSleeping-request.msg.html

(cl:defclass <SetSleeping-request> (roslisp-msg-protocol:ros-message)
  ((set_sleeping
    :reader set_sleeping
    :initarg :set_sleeping
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetSleeping-request (<SetSleeping-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSleeping-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSleeping-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetSleeping-request> is deprecated: use camera_control_msgs-srv:SetSleeping-request instead.")))

(cl:ensure-generic-function 'set_sleeping-val :lambda-list '(m))
(cl:defmethod set_sleeping-val ((m <SetSleeping-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:set_sleeping-val is deprecated.  Use camera_control_msgs-srv:set_sleeping instead.")
  (set_sleeping m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSleeping-request>) ostream)
  "Serializes a message object of type '<SetSleeping-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'set_sleeping) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSleeping-request>) istream)
  "Deserializes a message object of type '<SetSleeping-request>"
    (cl:setf (cl:slot-value msg 'set_sleeping) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSleeping-request>)))
  "Returns string type for a service object of type '<SetSleeping-request>"
  "camera_control_msgs/SetSleepingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSleeping-request)))
  "Returns string type for a service object of type 'SetSleeping-request"
  "camera_control_msgs/SetSleepingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSleeping-request>)))
  "Returns md5sum for a message object of type '<SetSleeping-request>"
  "f51754709312645db79ada18bc0254a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSleeping-request)))
  "Returns md5sum for a message object of type 'SetSleeping-request"
  "f51754709312645db79ada18bc0254a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSleeping-request>)))
  "Returns full string definition for message of type '<SetSleeping-request>"
  (cl:format cl:nil "# If the camera runs in topic mode (continuously publishing images over the~%# topics respecting the desired frame_rate) this service offers the posibillity~%# to pause the image acquisition. To restart the grabbing, this service should~%# be called again with set_sleeping set to false~%bool set_sleeping~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSleeping-request)))
  "Returns full string definition for message of type 'SetSleeping-request"
  (cl:format cl:nil "# If the camera runs in topic mode (continuously publishing images over the~%# topics respecting the desired frame_rate) this service offers the posibillity~%# to pause the image acquisition. To restart the grabbing, this service should~%# be called again with set_sleeping set to false~%bool set_sleeping~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSleeping-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSleeping-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSleeping-request
    (cl:cons ':set_sleeping (set_sleeping msg))
))
;//! \htmlinclude SetSleeping-response.msg.html

(cl:defclass <SetSleeping-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetSleeping-response (<SetSleeping-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetSleeping-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetSleeping-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetSleeping-response> is deprecated: use camera_control_msgs-srv:SetSleeping-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetSleeping-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:success-val is deprecated.  Use camera_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetSleeping-response>) ostream)
  "Serializes a message object of type '<SetSleeping-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetSleeping-response>) istream)
  "Deserializes a message object of type '<SetSleeping-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetSleeping-response>)))
  "Returns string type for a service object of type '<SetSleeping-response>"
  "camera_control_msgs/SetSleepingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSleeping-response)))
  "Returns string type for a service object of type 'SetSleeping-response"
  "camera_control_msgs/SetSleepingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetSleeping-response>)))
  "Returns md5sum for a message object of type '<SetSleeping-response>"
  "f51754709312645db79ada18bc0254a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetSleeping-response)))
  "Returns md5sum for a message object of type 'SetSleeping-response"
  "f51754709312645db79ada18bc0254a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetSleeping-response>)))
  "Returns full string definition for message of type '<SetSleeping-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetSleeping-response)))
  "Returns full string definition for message of type 'SetSleeping-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetSleeping-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetSleeping-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetSleeping-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetSleeping)))
  'SetSleeping-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetSleeping)))
  'SetSleeping-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetSleeping)))
  "Returns string type for a service object of type '<SetSleeping>"
  "camera_control_msgs/SetSleeping")