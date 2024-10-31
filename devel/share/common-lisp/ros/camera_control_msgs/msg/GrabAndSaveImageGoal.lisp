; Auto-generated. Do not edit!


(cl:in-package camera_control_msgs-msg)


;//! \htmlinclude GrabAndSaveImageGoal.msg.html

(cl:defclass <GrabAndSaveImageGoal> (roslisp-msg-protocol:ros-message)
  ((exposure_given
    :reader exposure_given
    :initarg :exposure_given
    :type cl:boolean
    :initform cl:nil)
   (exposure_time
    :reader exposure_time
    :initarg :exposure_time
    :type cl:float
    :initform 0.0)
   (gain_given
    :reader gain_given
    :initarg :gain_given
    :type cl:boolean
    :initform cl:nil)
   (gain_value
    :reader gain_value
    :initarg :gain_value
    :type cl:float
    :initform 0.0)
   (gamma_given
    :reader gamma_given
    :initarg :gamma_given
    :type cl:boolean
    :initform cl:nil)
   (gamma_value
    :reader gamma_value
    :initarg :gamma_value
    :type cl:float
    :initform 0.0)
   (brightness_given
    :reader brightness_given
    :initarg :brightness_given
    :type cl:boolean
    :initform cl:nil)
   (brightness_value
    :reader brightness_value
    :initarg :brightness_value
    :type cl:float
    :initform 0.0)
   (exposure_auto
    :reader exposure_auto
    :initarg :exposure_auto
    :type cl:boolean
    :initform cl:nil)
   (gain_auto
    :reader gain_auto
    :initarg :gain_auto
    :type cl:boolean
    :initform cl:nil)
   (img_storage_path_and_name
    :reader img_storage_path_and_name
    :initarg :img_storage_path_and_name
    :type cl:string
    :initform ""))
)

(cl:defclass GrabAndSaveImageGoal (<GrabAndSaveImageGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GrabAndSaveImageGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GrabAndSaveImageGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-msg:<GrabAndSaveImageGoal> is deprecated: use camera_control_msgs-msg:GrabAndSaveImageGoal instead.")))

(cl:ensure-generic-function 'exposure_given-val :lambda-list '(m))
(cl:defmethod exposure_given-val ((m <GrabAndSaveImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:exposure_given-val is deprecated.  Use camera_control_msgs-msg:exposure_given instead.")
  (exposure_given m))

(cl:ensure-generic-function 'exposure_time-val :lambda-list '(m))
(cl:defmethod exposure_time-val ((m <GrabAndSaveImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:exposure_time-val is deprecated.  Use camera_control_msgs-msg:exposure_time instead.")
  (exposure_time m))

(cl:ensure-generic-function 'gain_given-val :lambda-list '(m))
(cl:defmethod gain_given-val ((m <GrabAndSaveImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gain_given-val is deprecated.  Use camera_control_msgs-msg:gain_given instead.")
  (gain_given m))

(cl:ensure-generic-function 'gain_value-val :lambda-list '(m))
(cl:defmethod gain_value-val ((m <GrabAndSaveImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gain_value-val is deprecated.  Use camera_control_msgs-msg:gain_value instead.")
  (gain_value m))

(cl:ensure-generic-function 'gamma_given-val :lambda-list '(m))
(cl:defmethod gamma_given-val ((m <GrabAndSaveImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gamma_given-val is deprecated.  Use camera_control_msgs-msg:gamma_given instead.")
  (gamma_given m))

(cl:ensure-generic-function 'gamma_value-val :lambda-list '(m))
(cl:defmethod gamma_value-val ((m <GrabAndSaveImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gamma_value-val is deprecated.  Use camera_control_msgs-msg:gamma_value instead.")
  (gamma_value m))

(cl:ensure-generic-function 'brightness_given-val :lambda-list '(m))
(cl:defmethod brightness_given-val ((m <GrabAndSaveImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:brightness_given-val is deprecated.  Use camera_control_msgs-msg:brightness_given instead.")
  (brightness_given m))

(cl:ensure-generic-function 'brightness_value-val :lambda-list '(m))
(cl:defmethod brightness_value-val ((m <GrabAndSaveImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:brightness_value-val is deprecated.  Use camera_control_msgs-msg:brightness_value instead.")
  (brightness_value m))

(cl:ensure-generic-function 'exposure_auto-val :lambda-list '(m))
(cl:defmethod exposure_auto-val ((m <GrabAndSaveImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:exposure_auto-val is deprecated.  Use camera_control_msgs-msg:exposure_auto instead.")
  (exposure_auto m))

(cl:ensure-generic-function 'gain_auto-val :lambda-list '(m))
(cl:defmethod gain_auto-val ((m <GrabAndSaveImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gain_auto-val is deprecated.  Use camera_control_msgs-msg:gain_auto instead.")
  (gain_auto m))

(cl:ensure-generic-function 'img_storage_path_and_name-val :lambda-list '(m))
(cl:defmethod img_storage_path_and_name-val ((m <GrabAndSaveImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:img_storage_path_and_name-val is deprecated.  Use camera_control_msgs-msg:img_storage_path_and_name instead.")
  (img_storage_path_and_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GrabAndSaveImageGoal>) ostream)
  "Serializes a message object of type '<GrabAndSaveImageGoal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'exposure_given) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'exposure_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gain_given) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gain_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gamma_given) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gamma_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'brightness_given) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'brightness_value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'exposure_auto) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gain_auto) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'img_storage_path_and_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'img_storage_path_and_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GrabAndSaveImageGoal>) istream)
  "Deserializes a message object of type '<GrabAndSaveImageGoal>"
    (cl:setf (cl:slot-value msg 'exposure_given) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'exposure_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'gain_given) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gain_value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'gamma_given) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gamma_value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'brightness_given) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'brightness_value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'exposure_auto) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'gain_auto) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'img_storage_path_and_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'img_storage_path_and_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GrabAndSaveImageGoal>)))
  "Returns string type for a message object of type '<GrabAndSaveImageGoal>"
  "camera_control_msgs/GrabAndSaveImageGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GrabAndSaveImageGoal)))
  "Returns string type for a message object of type 'GrabAndSaveImageGoal"
  "camera_control_msgs/GrabAndSaveImageGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GrabAndSaveImageGoal>)))
  "Returns md5sum for a message object of type '<GrabAndSaveImageGoal>"
  "c66e7613728d45278cbdc7c5a89f6e08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GrabAndSaveImageGoal)))
  "Returns md5sum for a message object of type 'GrabAndSaveImageGoal"
  "c66e7613728d45278cbdc7c5a89f6e08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GrabAndSaveImageGoal>)))
  "Returns full string definition for message of type '<GrabAndSaveImageGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# The 'GrabAndSaveImageAction' has a similar goal as the 'GrabImagesAction' in~%# case of only grabbing one image. It additionally contains a string describing~%# the full storage path of the image to be captured.~%# It wont provide the grabbed image via the result topic. Instead it only~%# returns a flag indicating the success.~%~%##########################################~%################## GOAL ##################~%##########################################~%~%# Flag which indicates if the exposure time is provided and hence should be~%# set before grabbing~%bool exposure_given~%~%# Only relevant, if exposure_given is true:~%# The target exposure time in microseconds. This values can be overriden from~%# the brightness search, in case that the flag exposure_fixed is not true.~%float32 exposure_time~%~%# Flag which indicates if the gain is provided and hence should be set before~%# grabbing~%bool gain_given~%~%# Only relevant, if gain_given is true:~%# The target gain in percent of the maximal value the camera supports.~%# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so~%# called 'device specific units'. This value can be overriden from the~%# brightness search, in case that the gain_fixed flag is set to false.~%float32 gain_value~%~%# Flag which indicates if the gamma value is provided and hence should be set~%# before grabbing~%bool gamma_given~%~%# Only relevant, if gain_given is true:~%# Gamma correction of pixel intensity.~%# Adjusts the brightness of the pixel values output by the camera's sensor~%# to account for a non-linearity in the human perception of brightness or~%# of the display system (such as CRT).~%float32 gamma_value~%~%# Flag which indicates if the brightness value is provided and hence should~%# be set before grabbing~%bool brightness_given~%~%# Only relevant, if brightness_given is true:~%# The average intensity value of the resulting image. It depends the exposure~%# time as well as the gain setting.~%float32 brightness_value~%~%# Only relevant, if brightness_given is true:~%# If the camera should try reach the desired brightness, at least one of the~%# following flags MUST be set. If both are set, the interface will use the~%# profile that tries to keep the gain at minimum to reduce white noise.~%# 'exposure_auto' will adapt the exposure time to reach the brightness, wheras~%# 'gain_auto' does so by adapting the gain. If one of these flags is set to~%# false, the connected property will be kept fix.~%# In most of the cases trying to reach a target brightness only by varying the~%# gain and keeping the exposure time fix is not a good approach, because the~%# exposure range is many times higher than the gain range.~%bool exposure_auto~%bool gain_auto~%~%# File path and image name (including file-extension) to store the grabbed~%# image~%string img_storage_path_and_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GrabAndSaveImageGoal)))
  "Returns full string definition for message of type 'GrabAndSaveImageGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# The 'GrabAndSaveImageAction' has a similar goal as the 'GrabImagesAction' in~%# case of only grabbing one image. It additionally contains a string describing~%# the full storage path of the image to be captured.~%# It wont provide the grabbed image via the result topic. Instead it only~%# returns a flag indicating the success.~%~%##########################################~%################## GOAL ##################~%##########################################~%~%# Flag which indicates if the exposure time is provided and hence should be~%# set before grabbing~%bool exposure_given~%~%# Only relevant, if exposure_given is true:~%# The target exposure time in microseconds. This values can be overriden from~%# the brightness search, in case that the flag exposure_fixed is not true.~%float32 exposure_time~%~%# Flag which indicates if the gain is provided and hence should be set before~%# grabbing~%bool gain_given~%~%# Only relevant, if gain_given is true:~%# The target gain in percent of the maximal value the camera supports.~%# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so~%# called 'device specific units'. This value can be overriden from the~%# brightness search, in case that the gain_fixed flag is set to false.~%float32 gain_value~%~%# Flag which indicates if the gamma value is provided and hence should be set~%# before grabbing~%bool gamma_given~%~%# Only relevant, if gain_given is true:~%# Gamma correction of pixel intensity.~%# Adjusts the brightness of the pixel values output by the camera's sensor~%# to account for a non-linearity in the human perception of brightness or~%# of the display system (such as CRT).~%float32 gamma_value~%~%# Flag which indicates if the brightness value is provided and hence should~%# be set before grabbing~%bool brightness_given~%~%# Only relevant, if brightness_given is true:~%# The average intensity value of the resulting image. It depends the exposure~%# time as well as the gain setting.~%float32 brightness_value~%~%# Only relevant, if brightness_given is true:~%# If the camera should try reach the desired brightness, at least one of the~%# following flags MUST be set. If both are set, the interface will use the~%# profile that tries to keep the gain at minimum to reduce white noise.~%# 'exposure_auto' will adapt the exposure time to reach the brightness, wheras~%# 'gain_auto' does so by adapting the gain. If one of these flags is set to~%# false, the connected property will be kept fix.~%# In most of the cases trying to reach a target brightness only by varying the~%# gain and keeping the exposure time fix is not a good approach, because the~%# exposure range is many times higher than the gain range.~%bool exposure_auto~%bool gain_auto~%~%# File path and image name (including file-extension) to store the grabbed~%# image~%string img_storage_path_and_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GrabAndSaveImageGoal>))
  (cl:+ 0
     1
     4
     1
     4
     1
     4
     1
     4
     1
     1
     4 (cl:length (cl:slot-value msg 'img_storage_path_and_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GrabAndSaveImageGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'GrabAndSaveImageGoal
    (cl:cons ':exposure_given (exposure_given msg))
    (cl:cons ':exposure_time (exposure_time msg))
    (cl:cons ':gain_given (gain_given msg))
    (cl:cons ':gain_value (gain_value msg))
    (cl:cons ':gamma_given (gamma_given msg))
    (cl:cons ':gamma_value (gamma_value msg))
    (cl:cons ':brightness_given (brightness_given msg))
    (cl:cons ':brightness_value (brightness_value msg))
    (cl:cons ':exposure_auto (exposure_auto msg))
    (cl:cons ':gain_auto (gain_auto msg))
    (cl:cons ':img_storage_path_and_name (img_storage_path_and_name msg))
))
