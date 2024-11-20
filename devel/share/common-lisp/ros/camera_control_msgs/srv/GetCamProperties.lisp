; Auto-generated. Do not edit!


(cl:in-package camera_control_msgs-srv)


;//! \htmlinclude GetCamProperties-request.msg.html

(cl:defclass <GetCamProperties-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetCamProperties-request (<GetCamProperties-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCamProperties-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCamProperties-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<GetCamProperties-request> is deprecated: use camera_control_msgs-srv:GetCamProperties-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCamProperties-request>) ostream)
  "Serializes a message object of type '<GetCamProperties-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCamProperties-request>) istream)
  "Deserializes a message object of type '<GetCamProperties-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCamProperties-request>)))
  "Returns string type for a service object of type '<GetCamProperties-request>"
  "camera_control_msgs/GetCamPropertiesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCamProperties-request)))
  "Returns string type for a service object of type 'GetCamProperties-request"
  "camera_control_msgs/GetCamPropertiesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCamProperties-request>)))
  "Returns md5sum for a message object of type '<GetCamProperties-request>"
  "14bd64331efb0f665787f525f453c05d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCamProperties-request)))
  "Returns md5sum for a message object of type 'GetCamProperties-request"
  "14bd64331efb0f665787f525f453c05d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCamProperties-request>)))
  "Returns full string definition for message of type '<GetCamProperties-request>"
  (cl:format cl:nil "# This service returns the current camera properties as well as the limits~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCamProperties-request)))
  "Returns full string definition for message of type 'GetCamProperties-request"
  (cl:format cl:nil "# This service returns the current camera properties as well as the limits~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCamProperties-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCamProperties-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCamProperties-request
))
;//! \htmlinclude GetCamProperties-response.msg.html

(cl:defclass <GetCamProperties-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (is_sleeping
    :reader is_sleeping
    :initarg :is_sleeping
    :type cl:boolean
    :initform cl:nil)
   (device_user_id
    :reader device_user_id
    :initarg :device_user_id
    :type cl:string
    :initform "")
   (min_binning_x
    :reader min_binning_x
    :initarg :min_binning_x
    :type cl:integer
    :initform 0)
   (max_binning_x
    :reader max_binning_x
    :initarg :max_binning_x
    :type cl:integer
    :initform 0)
   (current_binning_x
    :reader current_binning_x
    :initarg :current_binning_x
    :type cl:integer
    :initform 0)
   (min_binning_y
    :reader min_binning_y
    :initarg :min_binning_y
    :type cl:integer
    :initform 0)
   (max_binning_y
    :reader max_binning_y
    :initarg :max_binning_y
    :type cl:integer
    :initform 0)
   (current_binning_y
    :reader current_binning_y
    :initarg :current_binning_y
    :type cl:integer
    :initform 0)
   (max_framerate
    :reader max_framerate
    :initarg :max_framerate
    :type cl:float
    :initform 0.0)
   (current_framerate
    :reader current_framerate
    :initarg :current_framerate
    :type cl:float
    :initform 0.0)
   (min_exposure
    :reader min_exposure
    :initarg :min_exposure
    :type cl:float
    :initform 0.0)
   (max_exposure
    :reader max_exposure
    :initarg :max_exposure
    :type cl:float
    :initform 0.0)
   (current_exposure
    :reader current_exposure
    :initarg :current_exposure
    :type cl:float
    :initform 0.0)
   (min_gain_in_cam_units
    :reader min_gain_in_cam_units
    :initarg :min_gain_in_cam_units
    :type cl:float
    :initform 0.0)
   (max_gain_in_cam_units
    :reader max_gain_in_cam_units
    :initarg :max_gain_in_cam_units
    :type cl:float
    :initform 0.0)
   (current_gain_in_cam_units
    :reader current_gain_in_cam_units
    :initarg :current_gain_in_cam_units
    :type cl:float
    :initform 0.0)
   (min_gain
    :reader min_gain
    :initarg :min_gain
    :type cl:float
    :initform 0.0)
   (max_gain
    :reader max_gain
    :initarg :max_gain
    :type cl:float
    :initform 0.0)
   (current_gain
    :reader current_gain
    :initarg :current_gain
    :type cl:float
    :initform 0.0)
   (min_gamma
    :reader min_gamma
    :initarg :min_gamma
    :type cl:float
    :initform 0.0)
   (max_gamma
    :reader max_gamma
    :initarg :max_gamma
    :type cl:float
    :initform 0.0)
   (current_gamma
    :reader current_gamma
    :initarg :current_gamma
    :type cl:float
    :initform 0.0)
   (brightness_continuous
    :reader brightness_continuous
    :initarg :brightness_continuous
    :type cl:boolean
    :initform cl:nil)
   (gain_auto
    :reader gain_auto
    :initarg :gain_auto
    :type cl:boolean
    :initform cl:nil)
   (exposure_auto
    :reader exposure_auto
    :initarg :exposure_auto
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetCamProperties-response (<GetCamProperties-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCamProperties-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCamProperties-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<GetCamProperties-response> is deprecated: use camera_control_msgs-srv:GetCamProperties-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:success-val is deprecated.  Use camera_control_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'is_sleeping-val :lambda-list '(m))
(cl:defmethod is_sleeping-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:is_sleeping-val is deprecated.  Use camera_control_msgs-srv:is_sleeping instead.")
  (is_sleeping m))

(cl:ensure-generic-function 'device_user_id-val :lambda-list '(m))
(cl:defmethod device_user_id-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:device_user_id-val is deprecated.  Use camera_control_msgs-srv:device_user_id instead.")
  (device_user_id m))

(cl:ensure-generic-function 'min_binning_x-val :lambda-list '(m))
(cl:defmethod min_binning_x-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:min_binning_x-val is deprecated.  Use camera_control_msgs-srv:min_binning_x instead.")
  (min_binning_x m))

(cl:ensure-generic-function 'max_binning_x-val :lambda-list '(m))
(cl:defmethod max_binning_x-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:max_binning_x-val is deprecated.  Use camera_control_msgs-srv:max_binning_x instead.")
  (max_binning_x m))

(cl:ensure-generic-function 'current_binning_x-val :lambda-list '(m))
(cl:defmethod current_binning_x-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:current_binning_x-val is deprecated.  Use camera_control_msgs-srv:current_binning_x instead.")
  (current_binning_x m))

(cl:ensure-generic-function 'min_binning_y-val :lambda-list '(m))
(cl:defmethod min_binning_y-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:min_binning_y-val is deprecated.  Use camera_control_msgs-srv:min_binning_y instead.")
  (min_binning_y m))

(cl:ensure-generic-function 'max_binning_y-val :lambda-list '(m))
(cl:defmethod max_binning_y-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:max_binning_y-val is deprecated.  Use camera_control_msgs-srv:max_binning_y instead.")
  (max_binning_y m))

(cl:ensure-generic-function 'current_binning_y-val :lambda-list '(m))
(cl:defmethod current_binning_y-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:current_binning_y-val is deprecated.  Use camera_control_msgs-srv:current_binning_y instead.")
  (current_binning_y m))

(cl:ensure-generic-function 'max_framerate-val :lambda-list '(m))
(cl:defmethod max_framerate-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:max_framerate-val is deprecated.  Use camera_control_msgs-srv:max_framerate instead.")
  (max_framerate m))

(cl:ensure-generic-function 'current_framerate-val :lambda-list '(m))
(cl:defmethod current_framerate-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:current_framerate-val is deprecated.  Use camera_control_msgs-srv:current_framerate instead.")
  (current_framerate m))

(cl:ensure-generic-function 'min_exposure-val :lambda-list '(m))
(cl:defmethod min_exposure-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:min_exposure-val is deprecated.  Use camera_control_msgs-srv:min_exposure instead.")
  (min_exposure m))

(cl:ensure-generic-function 'max_exposure-val :lambda-list '(m))
(cl:defmethod max_exposure-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:max_exposure-val is deprecated.  Use camera_control_msgs-srv:max_exposure instead.")
  (max_exposure m))

(cl:ensure-generic-function 'current_exposure-val :lambda-list '(m))
(cl:defmethod current_exposure-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:current_exposure-val is deprecated.  Use camera_control_msgs-srv:current_exposure instead.")
  (current_exposure m))

(cl:ensure-generic-function 'min_gain_in_cam_units-val :lambda-list '(m))
(cl:defmethod min_gain_in_cam_units-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:min_gain_in_cam_units-val is deprecated.  Use camera_control_msgs-srv:min_gain_in_cam_units instead.")
  (min_gain_in_cam_units m))

(cl:ensure-generic-function 'max_gain_in_cam_units-val :lambda-list '(m))
(cl:defmethod max_gain_in_cam_units-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:max_gain_in_cam_units-val is deprecated.  Use camera_control_msgs-srv:max_gain_in_cam_units instead.")
  (max_gain_in_cam_units m))

(cl:ensure-generic-function 'current_gain_in_cam_units-val :lambda-list '(m))
(cl:defmethod current_gain_in_cam_units-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:current_gain_in_cam_units-val is deprecated.  Use camera_control_msgs-srv:current_gain_in_cam_units instead.")
  (current_gain_in_cam_units m))

(cl:ensure-generic-function 'min_gain-val :lambda-list '(m))
(cl:defmethod min_gain-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:min_gain-val is deprecated.  Use camera_control_msgs-srv:min_gain instead.")
  (min_gain m))

(cl:ensure-generic-function 'max_gain-val :lambda-list '(m))
(cl:defmethod max_gain-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:max_gain-val is deprecated.  Use camera_control_msgs-srv:max_gain instead.")
  (max_gain m))

(cl:ensure-generic-function 'current_gain-val :lambda-list '(m))
(cl:defmethod current_gain-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:current_gain-val is deprecated.  Use camera_control_msgs-srv:current_gain instead.")
  (current_gain m))

(cl:ensure-generic-function 'min_gamma-val :lambda-list '(m))
(cl:defmethod min_gamma-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:min_gamma-val is deprecated.  Use camera_control_msgs-srv:min_gamma instead.")
  (min_gamma m))

(cl:ensure-generic-function 'max_gamma-val :lambda-list '(m))
(cl:defmethod max_gamma-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:max_gamma-val is deprecated.  Use camera_control_msgs-srv:max_gamma instead.")
  (max_gamma m))

(cl:ensure-generic-function 'current_gamma-val :lambda-list '(m))
(cl:defmethod current_gamma-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:current_gamma-val is deprecated.  Use camera_control_msgs-srv:current_gamma instead.")
  (current_gamma m))

(cl:ensure-generic-function 'brightness_continuous-val :lambda-list '(m))
(cl:defmethod brightness_continuous-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:brightness_continuous-val is deprecated.  Use camera_control_msgs-srv:brightness_continuous instead.")
  (brightness_continuous m))

(cl:ensure-generic-function 'gain_auto-val :lambda-list '(m))
(cl:defmethod gain_auto-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:gain_auto-val is deprecated.  Use camera_control_msgs-srv:gain_auto instead.")
  (gain_auto m))

(cl:ensure-generic-function 'exposure_auto-val :lambda-list '(m))
(cl:defmethod exposure_auto-val ((m <GetCamProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:exposure_auto-val is deprecated.  Use camera_control_msgs-srv:exposure_auto instead.")
  (exposure_auto m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCamProperties-response>) ostream)
  "Serializes a message object of type '<GetCamProperties-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_sleeping) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'device_user_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'device_user_id))
  (cl:let* ((signed (cl:slot-value msg 'min_binning_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'max_binning_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_binning_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'min_binning_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'max_binning_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_binning_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_framerate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_framerate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_exposure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_exposure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_exposure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_gain_in_cam_units))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_gain_in_cam_units))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_gain_in_cam_units))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_gamma))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_gamma))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_gamma))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'brightness_continuous) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gain_auto) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'exposure_auto) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCamProperties-response>) istream)
  "Deserializes a message object of type '<GetCamProperties-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_sleeping) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'device_user_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'device_user_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'min_binning_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'max_binning_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_binning_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'min_binning_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'max_binning_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_binning_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_framerate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_framerate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_exposure) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_exposure) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_exposure) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_gain_in_cam_units) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_gain_in_cam_units) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_gain_in_cam_units) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_gain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_gain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_gain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_gamma) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_gamma) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_gamma) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'brightness_continuous) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'gain_auto) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'exposure_auto) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCamProperties-response>)))
  "Returns string type for a service object of type '<GetCamProperties-response>"
  "camera_control_msgs/GetCamPropertiesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCamProperties-response)))
  "Returns string type for a service object of type 'GetCamProperties-response"
  "camera_control_msgs/GetCamPropertiesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCamProperties-response>)))
  "Returns md5sum for a message object of type '<GetCamProperties-response>"
  "14bd64331efb0f665787f525f453c05d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCamProperties-response)))
  "Returns md5sum for a message object of type 'GetCamProperties-response"
  "14bd64331efb0f665787f525f453c05d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCamProperties-response>)))
  "Returns full string definition for message of type '<GetCamProperties-response>"
  (cl:format cl:nil "bool success~%~%bool is_sleeping~%~%string device_user_id~%~%int32 min_binning_x~%int32 max_binning_x~%int32 current_binning_x~%~%int32 min_binning_y~%int32 max_binning_y~%int32 current_binning_y~%~%float32 max_framerate~%float32 current_framerate~%~%float32 min_exposure~%float32 max_exposure~%float32 current_exposure~%~%float32 min_gain_in_cam_units~%float32 max_gain_in_cam_units~%float32 current_gain_in_cam_units~%~%float32 min_gain~%float32 max_gain~%float32 current_gain~%~%float32 min_gamma~%float32 max_gamma~%float32 current_gamma~%~%bool brightness_continuous~%bool gain_auto~%bool exposure_auto~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCamProperties-response)))
  "Returns full string definition for message of type 'GetCamProperties-response"
  (cl:format cl:nil "bool success~%~%bool is_sleeping~%~%string device_user_id~%~%int32 min_binning_x~%int32 max_binning_x~%int32 current_binning_x~%~%int32 min_binning_y~%int32 max_binning_y~%int32 current_binning_y~%~%float32 max_framerate~%float32 current_framerate~%~%float32 min_exposure~%float32 max_exposure~%float32 current_exposure~%~%float32 min_gain_in_cam_units~%float32 max_gain_in_cam_units~%float32 current_gain_in_cam_units~%~%float32 min_gain~%float32 max_gain~%float32 current_gain~%~%float32 min_gamma~%float32 max_gamma~%float32 current_gamma~%~%bool brightness_continuous~%bool gain_auto~%bool exposure_auto~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCamProperties-response>))
  (cl:+ 0
     1
     1
     4 (cl:length (cl:slot-value msg 'device_user_id))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCamProperties-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCamProperties-response
    (cl:cons ':success (success msg))
    (cl:cons ':is_sleeping (is_sleeping msg))
    (cl:cons ':device_user_id (device_user_id msg))
    (cl:cons ':min_binning_x (min_binning_x msg))
    (cl:cons ':max_binning_x (max_binning_x msg))
    (cl:cons ':current_binning_x (current_binning_x msg))
    (cl:cons ':min_binning_y (min_binning_y msg))
    (cl:cons ':max_binning_y (max_binning_y msg))
    (cl:cons ':current_binning_y (current_binning_y msg))
    (cl:cons ':max_framerate (max_framerate msg))
    (cl:cons ':current_framerate (current_framerate msg))
    (cl:cons ':min_exposure (min_exposure msg))
    (cl:cons ':max_exposure (max_exposure msg))
    (cl:cons ':current_exposure (current_exposure msg))
    (cl:cons ':min_gain_in_cam_units (min_gain_in_cam_units msg))
    (cl:cons ':max_gain_in_cam_units (max_gain_in_cam_units msg))
    (cl:cons ':current_gain_in_cam_units (current_gain_in_cam_units msg))
    (cl:cons ':min_gain (min_gain msg))
    (cl:cons ':max_gain (max_gain msg))
    (cl:cons ':current_gain (current_gain msg))
    (cl:cons ':min_gamma (min_gamma msg))
    (cl:cons ':max_gamma (max_gamma msg))
    (cl:cons ':current_gamma (current_gamma msg))
    (cl:cons ':brightness_continuous (brightness_continuous msg))
    (cl:cons ':gain_auto (gain_auto msg))
    (cl:cons ':exposure_auto (exposure_auto msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCamProperties)))
  'GetCamProperties-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCamProperties)))
  'GetCamProperties-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCamProperties)))
  "Returns string type for a service object of type '<GetCamProperties>"
  "camera_control_msgs/GetCamProperties")