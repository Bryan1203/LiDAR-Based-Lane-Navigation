; Auto-generated. Do not edit!


(cl:in-package camera_control_msgs-srv)


;//! \htmlinclude SetBinning-request.msg.html

(cl:defclass <SetBinning-request> (roslisp-msg-protocol:ros-message)
  ((target_binning_x
    :reader target_binning_x
    :initarg :target_binning_x
    :type cl:integer
    :initform 0)
   (target_binning_y
    :reader target_binning_y
    :initarg :target_binning_y
    :type cl:integer
    :initform 0))
)

(cl:defclass SetBinning-request (<SetBinning-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBinning-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBinning-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetBinning-request> is deprecated: use camera_control_msgs-srv:SetBinning-request instead.")))

(cl:ensure-generic-function 'target_binning_x-val :lambda-list '(m))
(cl:defmethod target_binning_x-val ((m <SetBinning-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:target_binning_x-val is deprecated.  Use camera_control_msgs-srv:target_binning_x instead.")
  (target_binning_x m))

(cl:ensure-generic-function 'target_binning_y-val :lambda-list '(m))
(cl:defmethod target_binning_y-val ((m <SetBinning-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:target_binning_y-val is deprecated.  Use camera_control_msgs-srv:target_binning_y instead.")
  (target_binning_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBinning-request>) ostream)
  "Serializes a message object of type '<SetBinning-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_binning_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_binning_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'target_binning_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'target_binning_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_binning_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_binning_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'target_binning_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'target_binning_y)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBinning-request>) istream)
  "Deserializes a message object of type '<SetBinning-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_binning_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_binning_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'target_binning_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'target_binning_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_binning_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'target_binning_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'target_binning_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'target_binning_y)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBinning-request>)))
  "Returns string type for a service object of type '<SetBinning-request>"
  "camera_control_msgs/SetBinningRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBinning-request)))
  "Returns string type for a service object of type 'SetBinning-request"
  "camera_control_msgs/SetBinningRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBinning-request>)))
  "Returns md5sum for a message object of type '<SetBinning-request>"
  "fb7b82cb93bc5513ade2d601c7a7d975")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBinning-request)))
  "Returns md5sum for a message object of type 'SetBinning-request"
  "fb7b82cb93bc5513ade2d601c7a7d975")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBinning-request>)))
  "Returns full string definition for message of type '<SetBinning-request>"
  (cl:format cl:nil "# Binning factor to get downsampled images. It refers here to any camera~%# setting which combines rectangular neighborhoods of pixels into larger~%# \"super-pixels.\" It reduces the resolution of the output image to~%# (width / binning_x) x (height / binning_y).~%# The default values binning_x = binning_y = 0 are considered the same~%# as binning_x = binning_y = 1 (no subsampling).~%# Calling this service with target binning values will change the binning entry~%# in the published camera_info_msg~%uint32 target_binning_x~%uint32 target_binning_y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBinning-request)))
  "Returns full string definition for message of type 'SetBinning-request"
  (cl:format cl:nil "# Binning factor to get downsampled images. It refers here to any camera~%# setting which combines rectangular neighborhoods of pixels into larger~%# \"super-pixels.\" It reduces the resolution of the output image to~%# (width / binning_x) x (height / binning_y).~%# The default values binning_x = binning_y = 0 are considered the same~%# as binning_x = binning_y = 1 (no subsampling).~%# Calling this service with target binning values will change the binning entry~%# in the published camera_info_msg~%uint32 target_binning_x~%uint32 target_binning_y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBinning-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBinning-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBinning-request
    (cl:cons ':target_binning_x (target_binning_x msg))
    (cl:cons ':target_binning_y (target_binning_y msg))
))
;//! \htmlinclude SetBinning-response.msg.html

(cl:defclass <SetBinning-response> (roslisp-msg-protocol:ros-message)
  ((reached_binning_x
    :reader reached_binning_x
    :initarg :reached_binning_x
    :type cl:integer
    :initform 0)
   (reached_binning_y
    :reader reached_binning_y
    :initarg :reached_binning_y
    :type cl:integer
    :initform 0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetBinning-response (<SetBinning-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBinning-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBinning-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetBinning-response> is deprecated: use camera_control_msgs-srv:SetBinning-response instead.")))

(cl:ensure-generic-function 'reached_binning_x-val :lambda-list '(m))
(cl:defmethod reached_binning_x-val ((m <SetBinning-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:reached_binning_x-val is deprecated.  Use camera_control_msgs-srv:reached_binning_x instead.")
  (reached_binning_x m))

(cl:ensure-generic-function 'reached_binning_y-val :lambda-list '(m))
(cl:defmethod reached_binning_y-val ((m <SetBinning-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:reached_binning_y-val is deprecated.  Use camera_control_msgs-srv:reached_binning_y instead.")
  (reached_binning_y m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetBinning-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:success-val is deprecated.  Use camera_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBinning-response>) ostream)
  "Serializes a message object of type '<SetBinning-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reached_binning_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reached_binning_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reached_binning_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reached_binning_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reached_binning_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reached_binning_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reached_binning_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reached_binning_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBinning-response>) istream)
  "Deserializes a message object of type '<SetBinning-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reached_binning_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reached_binning_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reached_binning_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reached_binning_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reached_binning_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reached_binning_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'reached_binning_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'reached_binning_y)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBinning-response>)))
  "Returns string type for a service object of type '<SetBinning-response>"
  "camera_control_msgs/SetBinningResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBinning-response)))
  "Returns string type for a service object of type 'SetBinning-response"
  "camera_control_msgs/SetBinningResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBinning-response>)))
  "Returns md5sum for a message object of type '<SetBinning-response>"
  "fb7b82cb93bc5513ade2d601c7a7d975")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBinning-response)))
  "Returns md5sum for a message object of type 'SetBinning-response"
  "fb7b82cb93bc5513ade2d601c7a7d975")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBinning-response>)))
  "Returns full string definition for message of type '<SetBinning-response>"
  (cl:format cl:nil "# Exact match can not always be reached~%uint32 reached_binning_x~%uint32 reached_binning_y~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBinning-response)))
  "Returns full string definition for message of type 'SetBinning-response"
  (cl:format cl:nil "# Exact match can not always be reached~%uint32 reached_binning_x~%uint32 reached_binning_y~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBinning-response>))
  (cl:+ 0
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBinning-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBinning-response
    (cl:cons ':reached_binning_x (reached_binning_x msg))
    (cl:cons ':reached_binning_y (reached_binning_y msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetBinning)))
  'SetBinning-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetBinning)))
  'SetBinning-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBinning)))
  "Returns string type for a service object of type '<SetBinning>"
  "camera_control_msgs/SetBinning")