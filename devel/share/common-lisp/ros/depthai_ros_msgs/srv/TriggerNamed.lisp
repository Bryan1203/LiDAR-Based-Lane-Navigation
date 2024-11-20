; Auto-generated. Do not edit!


(cl:in-package depthai_ros_msgs-srv)


;//! \htmlinclude TriggerNamed-request.msg.html

(cl:defclass <TriggerNamed-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass TriggerNamed-request (<TriggerNamed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TriggerNamed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TriggerNamed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name depthai_ros_msgs-srv:<TriggerNamed-request> is deprecated: use depthai_ros_msgs-srv:TriggerNamed-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <TriggerNamed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-srv:name-val is deprecated.  Use depthai_ros_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TriggerNamed-request>) ostream)
  "Serializes a message object of type '<TriggerNamed-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TriggerNamed-request>) istream)
  "Deserializes a message object of type '<TriggerNamed-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TriggerNamed-request>)))
  "Returns string type for a service object of type '<TriggerNamed-request>"
  "depthai_ros_msgs/TriggerNamedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerNamed-request)))
  "Returns string type for a service object of type 'TriggerNamed-request"
  "depthai_ros_msgs/TriggerNamedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TriggerNamed-request>)))
  "Returns md5sum for a message object of type '<TriggerNamed-request>"
  "d82dc6474dd88dad5e1615ab1b2ca74c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TriggerNamed-request)))
  "Returns md5sum for a message object of type 'TriggerNamed-request"
  "d82dc6474dd88dad5e1615ab1b2ca74c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TriggerNamed-request>)))
  "Returns full string definition for message of type '<TriggerNamed-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TriggerNamed-request)))
  "Returns full string definition for message of type 'TriggerNamed-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TriggerNamed-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TriggerNamed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TriggerNamed-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude TriggerNamed-response.msg.html

(cl:defclass <TriggerNamed-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass TriggerNamed-response (<TriggerNamed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TriggerNamed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TriggerNamed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name depthai_ros_msgs-srv:<TriggerNamed-response> is deprecated: use depthai_ros_msgs-srv:TriggerNamed-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <TriggerNamed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-srv:success-val is deprecated.  Use depthai_ros_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <TriggerNamed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-srv:message-val is deprecated.  Use depthai_ros_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TriggerNamed-response>) ostream)
  "Serializes a message object of type '<TriggerNamed-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TriggerNamed-response>) istream)
  "Deserializes a message object of type '<TriggerNamed-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TriggerNamed-response>)))
  "Returns string type for a service object of type '<TriggerNamed-response>"
  "depthai_ros_msgs/TriggerNamedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerNamed-response)))
  "Returns string type for a service object of type 'TriggerNamed-response"
  "depthai_ros_msgs/TriggerNamedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TriggerNamed-response>)))
  "Returns md5sum for a message object of type '<TriggerNamed-response>"
  "d82dc6474dd88dad5e1615ab1b2ca74c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TriggerNamed-response)))
  "Returns md5sum for a message object of type 'TriggerNamed-response"
  "d82dc6474dd88dad5e1615ab1b2ca74c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TriggerNamed-response>)))
  "Returns full string definition for message of type '<TriggerNamed-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TriggerNamed-response)))
  "Returns full string definition for message of type 'TriggerNamed-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TriggerNamed-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TriggerNamed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TriggerNamed-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TriggerNamed)))
  'TriggerNamed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TriggerNamed)))
  'TriggerNamed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerNamed)))
  "Returns string type for a service object of type '<TriggerNamed>"
  "depthai_ros_msgs/TriggerNamed")