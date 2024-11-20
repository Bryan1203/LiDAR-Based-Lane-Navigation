; Auto-generated. Do not edit!


(cl:in-package depthai_ros_msgs-msg)


;//! \htmlinclude AutoFocusCtrl.msg.html

(cl:defclass <AutoFocusCtrl> (roslisp-msg-protocol:ros-message)
  ((auto_focus_mode
    :reader auto_focus_mode
    :initarg :auto_focus_mode
    :type cl:fixnum
    :initform 0)
   (trigger_auto_focus
    :reader trigger_auto_focus
    :initarg :trigger_auto_focus
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AutoFocusCtrl (<AutoFocusCtrl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AutoFocusCtrl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AutoFocusCtrl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name depthai_ros_msgs-msg:<AutoFocusCtrl> is deprecated: use depthai_ros_msgs-msg:AutoFocusCtrl instead.")))

(cl:ensure-generic-function 'auto_focus_mode-val :lambda-list '(m))
(cl:defmethod auto_focus_mode-val ((m <AutoFocusCtrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:auto_focus_mode-val is deprecated.  Use depthai_ros_msgs-msg:auto_focus_mode instead.")
  (auto_focus_mode m))

(cl:ensure-generic-function 'trigger_auto_focus-val :lambda-list '(m))
(cl:defmethod trigger_auto_focus-val ((m <AutoFocusCtrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader depthai_ros_msgs-msg:trigger_auto_focus-val is deprecated.  Use depthai_ros_msgs-msg:trigger_auto_focus instead.")
  (trigger_auto_focus m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AutoFocusCtrl>)))
    "Constants for message type '<AutoFocusCtrl>"
  '((:AF_MODE_AUTO . 0)
    (:AF_MODE_MACRO . 1)
    (:AF_MODE_CONTINUOUS_VIDEO . 2)
    (:AF_MODE_CONTINUOUS_PICTURE . 3)
    (:AF_MODE_EDOF . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AutoFocusCtrl)))
    "Constants for message type 'AutoFocusCtrl"
  '((:AF_MODE_AUTO . 0)
    (:AF_MODE_MACRO . 1)
    (:AF_MODE_CONTINUOUS_VIDEO . 2)
    (:AF_MODE_CONTINUOUS_PICTURE . 3)
    (:AF_MODE_EDOF . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AutoFocusCtrl>) ostream)
  "Serializes a message object of type '<AutoFocusCtrl>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'auto_focus_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'trigger_auto_focus) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AutoFocusCtrl>) istream)
  "Deserializes a message object of type '<AutoFocusCtrl>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'auto_focus_mode)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'trigger_auto_focus) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AutoFocusCtrl>)))
  "Returns string type for a message object of type '<AutoFocusCtrl>"
  "depthai_ros_msgs/AutoFocusCtrl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AutoFocusCtrl)))
  "Returns string type for a message object of type 'AutoFocusCtrl"
  "depthai_ros_msgs/AutoFocusCtrl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AutoFocusCtrl>)))
  "Returns md5sum for a message object of type '<AutoFocusCtrl>"
  "57bd33867f64daa701fce3a114856f8f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AutoFocusCtrl)))
  "Returns md5sum for a message object of type 'AutoFocusCtrl"
  "57bd33867f64daa701fce3a114856f8f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AutoFocusCtrl>)))
  "Returns full string definition for message of type '<AutoFocusCtrl>"
  (cl:format cl:nil "uint8 AF_MODE_AUTO = 0~%uint8 AF_MODE_MACRO = 1~%uint8 AF_MODE_CONTINUOUS_VIDEO = 2~%uint8 AF_MODE_CONTINUOUS_PICTURE = 3~%uint8 AF_MODE_EDOF = 4~%~%uint8 auto_focus_mode~%~%bool trigger_auto_focus~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AutoFocusCtrl)))
  "Returns full string definition for message of type 'AutoFocusCtrl"
  (cl:format cl:nil "uint8 AF_MODE_AUTO = 0~%uint8 AF_MODE_MACRO = 1~%uint8 AF_MODE_CONTINUOUS_VIDEO = 2~%uint8 AF_MODE_CONTINUOUS_PICTURE = 3~%uint8 AF_MODE_EDOF = 4~%~%uint8 auto_focus_mode~%~%bool trigger_auto_focus~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AutoFocusCtrl>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AutoFocusCtrl>))
  "Converts a ROS message object to a list"
  (cl:list 'AutoFocusCtrl
    (cl:cons ':auto_focus_mode (auto_focus_mode msg))
    (cl:cons ':trigger_auto_focus (trigger_auto_focus msg))
))
