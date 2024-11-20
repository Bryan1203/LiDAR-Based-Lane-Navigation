; Auto-generated. Do not edit!


(cl:in-package camera_control_msgs-srv)


;//! \htmlinclude SetBrightness-request.msg.html

(cl:defclass <SetBrightness-request> (roslisp-msg-protocol:ros-message)
  ((target_brightness
    :reader target_brightness
    :initarg :target_brightness
    :type cl:integer
    :initform 0)
   (brightness_continuous
    :reader brightness_continuous
    :initarg :brightness_continuous
    :type cl:boolean
    :initform cl:nil)
   (exposure_auto
    :reader exposure_auto
    :initarg :exposure_auto
    :type cl:boolean
    :initform cl:nil)
   (gain_auto
    :reader gain_auto
    :initarg :gain_auto
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetBrightness-request (<SetBrightness-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBrightness-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBrightness-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetBrightness-request> is deprecated: use camera_control_msgs-srv:SetBrightness-request instead.")))

(cl:ensure-generic-function 'target_brightness-val :lambda-list '(m))
(cl:defmethod target_brightness-val ((m <SetBrightness-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:target_brightness-val is deprecated.  Use camera_control_msgs-srv:target_brightness instead.")
  (target_brightness m))

(cl:ensure-generic-function 'brightness_continuous-val :lambda-list '(m))
(cl:defmethod brightness_continuous-val ((m <SetBrightness-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:brightness_continuous-val is deprecated.  Use camera_control_msgs-srv:brightness_continuous instead.")
  (brightness_continuous m))

(cl:ensure-generic-function 'exposure_auto-val :lambda-list '(m))
(cl:defmethod exposure_auto-val ((m <SetBrightness-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:exposure_auto-val is deprecated.  Use camera_control_msgs-srv:exposure_auto instead.")
  (exposure_auto m))

(cl:ensure-generic-function 'gain_auto-val :lambda-list '(m))
(cl:defmethod gain_auto-val ((m <SetBrightness-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:gain_auto-val is deprecated.  Use camera_control_msgs-srv:gain_auto instead.")
  (gain_auto m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBrightness-request>) ostream)
  "Serializes a message object of type '<SetBrightness-request>"
  (cl:let* ((signed (cl:slot-value msg 'target_brightness)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'brightness_continuous) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'exposure_auto) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gain_auto) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBrightness-request>) istream)
  "Deserializes a message object of type '<SetBrightness-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_brightness) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'brightness_continuous) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'exposure_auto) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'gain_auto) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBrightness-request>)))
  "Returns string type for a service object of type '<SetBrightness-request>"
  "camera_control_msgs/SetBrightnessRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBrightness-request)))
  "Returns string type for a service object of type 'SetBrightness-request"
  "camera_control_msgs/SetBrightnessRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBrightness-request>)))
  "Returns md5sum for a message object of type '<SetBrightness-request>"
  "36481c45e92096457660dc69b68eb6b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBrightness-request)))
  "Returns md5sum for a message object of type 'SetBrightness-request"
  "36481c45e92096457660dc69b68eb6b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBrightness-request>)))
  "Returns full string definition for message of type '<SetBrightness-request>"
  (cl:format cl:nil "# The target brightness, which is average intensity values of the images.~%# It depends the exposure time as well as the gain setting.~%int32 target_brightness~%~%# The brightness_continuous flag controls the auto brightness function.~%# If it is set to false, the given brightness will only be reached once.~%# Hence changing light conditions lead to changing brightness values.~%# If it is set to true, the given brightness will be reached continuously,~%# trying to adapt to changing light conditions. The 'brightness_contunuous'~%# mode is is only possible for values in the possible auto range of the arena~%# API which is e.g. [50 - 205] for acA2500-14um and acA1920-40gm~%bool brightness_continuous~%~%# If the camera should try reach or keep the desired brightness, hence adapting~%# to changing light conditions, at least one of the following flags MUST be set.~%# If both are set, the interface will use the profile that tries to keep the~%# gain at minimum to reduce white noise.~%# 'exposure_auto' will adapt the exposure time to reach the brightness, wheras~%# 'gain_auto' does so by adapting the gain.~%bool exposure_auto~%bool gain_auto~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBrightness-request)))
  "Returns full string definition for message of type 'SetBrightness-request"
  (cl:format cl:nil "# The target brightness, which is average intensity values of the images.~%# It depends the exposure time as well as the gain setting.~%int32 target_brightness~%~%# The brightness_continuous flag controls the auto brightness function.~%# If it is set to false, the given brightness will only be reached once.~%# Hence changing light conditions lead to changing brightness values.~%# If it is set to true, the given brightness will be reached continuously,~%# trying to adapt to changing light conditions. The 'brightness_contunuous'~%# mode is is only possible for values in the possible auto range of the arena~%# API which is e.g. [50 - 205] for acA2500-14um and acA1920-40gm~%bool brightness_continuous~%~%# If the camera should try reach or keep the desired brightness, hence adapting~%# to changing light conditions, at least one of the following flags MUST be set.~%# If both are set, the interface will use the profile that tries to keep the~%# gain at minimum to reduce white noise.~%# 'exposure_auto' will adapt the exposure time to reach the brightness, wheras~%# 'gain_auto' does so by adapting the gain.~%bool exposure_auto~%bool gain_auto~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBrightness-request>))
  (cl:+ 0
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBrightness-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBrightness-request
    (cl:cons ':target_brightness (target_brightness msg))
    (cl:cons ':brightness_continuous (brightness_continuous msg))
    (cl:cons ':exposure_auto (exposure_auto msg))
    (cl:cons ':gain_auto (gain_auto msg))
))
;//! \htmlinclude SetBrightness-response.msg.html

(cl:defclass <SetBrightness-response> (roslisp-msg-protocol:ros-message)
  ((reached_brightness
    :reader reached_brightness
    :initarg :reached_brightness
    :type cl:integer
    :initform 0)
   (reached_exposure_time
    :reader reached_exposure_time
    :initarg :reached_exposure_time
    :type cl:float
    :initform 0.0)
   (reached_gain_value
    :reader reached_gain_value
    :initarg :reached_gain_value
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetBrightness-response (<SetBrightness-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBrightness-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBrightness-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetBrightness-response> is deprecated: use camera_control_msgs-srv:SetBrightness-response instead.")))

(cl:ensure-generic-function 'reached_brightness-val :lambda-list '(m))
(cl:defmethod reached_brightness-val ((m <SetBrightness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:reached_brightness-val is deprecated.  Use camera_control_msgs-srv:reached_brightness instead.")
  (reached_brightness m))

(cl:ensure-generic-function 'reached_exposure_time-val :lambda-list '(m))
(cl:defmethod reached_exposure_time-val ((m <SetBrightness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:reached_exposure_time-val is deprecated.  Use camera_control_msgs-srv:reached_exposure_time instead.")
  (reached_exposure_time m))

(cl:ensure-generic-function 'reached_gain_value-val :lambda-list '(m))
(cl:defmethod reached_gain_value-val ((m <SetBrightness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:reached_gain_value-val is deprecated.  Use camera_control_msgs-srv:reached_gain_value instead.")
  (reached_gain_value m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetBrightness-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:success-val is deprecated.  Use camera_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBrightness-response>) ostream)
  "Serializes a message object of type '<SetBrightness-response>"
  (cl:let* ((signed (cl:slot-value msg 'reached_brightness)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'reached_exposure_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'reached_gain_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBrightness-response>) istream)
  "Deserializes a message object of type '<SetBrightness-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reached_brightness) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reached_exposure_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reached_gain_value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBrightness-response>)))
  "Returns string type for a service object of type '<SetBrightness-response>"
  "camera_control_msgs/SetBrightnessResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBrightness-response)))
  "Returns string type for a service object of type 'SetBrightness-response"
  "camera_control_msgs/SetBrightnessResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBrightness-response>)))
  "Returns md5sum for a message object of type '<SetBrightness-response>"
  "36481c45e92096457660dc69b68eb6b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBrightness-response)))
  "Returns md5sum for a message object of type 'SetBrightness-response"
  "36481c45e92096457660dc69b68eb6b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBrightness-response>)))
  "Returns full string definition for message of type '<SetBrightness-response>"
  (cl:format cl:nil "~%# Exact match can not always be reached~%int32 reached_brightness~%float32 reached_exposure_time~%float32 reached_gain_value~%~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBrightness-response)))
  "Returns full string definition for message of type 'SetBrightness-response"
  (cl:format cl:nil "~%# Exact match can not always be reached~%int32 reached_brightness~%float32 reached_exposure_time~%float32 reached_gain_value~%~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBrightness-response>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBrightness-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBrightness-response
    (cl:cons ':reached_brightness (reached_brightness msg))
    (cl:cons ':reached_exposure_time (reached_exposure_time msg))
    (cl:cons ':reached_gain_value (reached_gain_value msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetBrightness)))
  'SetBrightness-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetBrightness)))
  'SetBrightness-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBrightness)))
  "Returns string type for a service object of type '<SetBrightness>"
  "camera_control_msgs/SetBrightness")