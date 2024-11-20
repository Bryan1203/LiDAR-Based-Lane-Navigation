; Auto-generated. Do not edit!


(cl:in-package camera_control_msgs-msg)


;//! \htmlinclude GrabImagesGoal.msg.html

(cl:defclass <GrabImagesGoal> (roslisp-msg-protocol:ros-message)
  ((exposure_given
    :reader exposure_given
    :initarg :exposure_given
    :type cl:boolean
    :initform cl:nil)
   (exposure_times
    :reader exposure_times
    :initarg :exposure_times
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (gain_given
    :reader gain_given
    :initarg :gain_given
    :type cl:boolean
    :initform cl:nil)
   (gain_values
    :reader gain_values
    :initarg :gain_values
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (gamma_given
    :reader gamma_given
    :initarg :gamma_given
    :type cl:boolean
    :initform cl:nil)
   (gamma_values
    :reader gamma_values
    :initarg :gamma_values
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (brightness_given
    :reader brightness_given
    :initarg :brightness_given
    :type cl:boolean
    :initform cl:nil)
   (brightness_values
    :reader brightness_values
    :initarg :brightness_values
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
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

(cl:defclass GrabImagesGoal (<GrabImagesGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GrabImagesGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GrabImagesGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-msg:<GrabImagesGoal> is deprecated: use camera_control_msgs-msg:GrabImagesGoal instead.")))

(cl:ensure-generic-function 'exposure_given-val :lambda-list '(m))
(cl:defmethod exposure_given-val ((m <GrabImagesGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:exposure_given-val is deprecated.  Use camera_control_msgs-msg:exposure_given instead.")
  (exposure_given m))

(cl:ensure-generic-function 'exposure_times-val :lambda-list '(m))
(cl:defmethod exposure_times-val ((m <GrabImagesGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:exposure_times-val is deprecated.  Use camera_control_msgs-msg:exposure_times instead.")
  (exposure_times m))

(cl:ensure-generic-function 'gain_given-val :lambda-list '(m))
(cl:defmethod gain_given-val ((m <GrabImagesGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gain_given-val is deprecated.  Use camera_control_msgs-msg:gain_given instead.")
  (gain_given m))

(cl:ensure-generic-function 'gain_values-val :lambda-list '(m))
(cl:defmethod gain_values-val ((m <GrabImagesGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gain_values-val is deprecated.  Use camera_control_msgs-msg:gain_values instead.")
  (gain_values m))

(cl:ensure-generic-function 'gamma_given-val :lambda-list '(m))
(cl:defmethod gamma_given-val ((m <GrabImagesGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gamma_given-val is deprecated.  Use camera_control_msgs-msg:gamma_given instead.")
  (gamma_given m))

(cl:ensure-generic-function 'gamma_values-val :lambda-list '(m))
(cl:defmethod gamma_values-val ((m <GrabImagesGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gamma_values-val is deprecated.  Use camera_control_msgs-msg:gamma_values instead.")
  (gamma_values m))

(cl:ensure-generic-function 'brightness_given-val :lambda-list '(m))
(cl:defmethod brightness_given-val ((m <GrabImagesGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:brightness_given-val is deprecated.  Use camera_control_msgs-msg:brightness_given instead.")
  (brightness_given m))

(cl:ensure-generic-function 'brightness_values-val :lambda-list '(m))
(cl:defmethod brightness_values-val ((m <GrabImagesGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:brightness_values-val is deprecated.  Use camera_control_msgs-msg:brightness_values instead.")
  (brightness_values m))

(cl:ensure-generic-function 'exposure_auto-val :lambda-list '(m))
(cl:defmethod exposure_auto-val ((m <GrabImagesGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:exposure_auto-val is deprecated.  Use camera_control_msgs-msg:exposure_auto instead.")
  (exposure_auto m))

(cl:ensure-generic-function 'gain_auto-val :lambda-list '(m))
(cl:defmethod gain_auto-val ((m <GrabImagesGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gain_auto-val is deprecated.  Use camera_control_msgs-msg:gain_auto instead.")
  (gain_auto m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GrabImagesGoal>) ostream)
  "Serializes a message object of type '<GrabImagesGoal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'exposure_given) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'exposure_times))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'exposure_times))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gain_given) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gain_values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'gain_values))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gamma_given) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gamma_values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'gamma_values))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'brightness_given) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'brightness_values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'brightness_values))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'exposure_auto) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gain_auto) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GrabImagesGoal>) istream)
  "Deserializes a message object of type '<GrabImagesGoal>"
    (cl:setf (cl:slot-value msg 'exposure_given) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'exposure_times) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'exposure_times)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'gain_given) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gain_values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gain_values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'gamma_given) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gamma_values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gamma_values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'brightness_given) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'brightness_values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'brightness_values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'exposure_auto) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'gain_auto) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GrabImagesGoal>)))
  "Returns string type for a message object of type '<GrabImagesGoal>"
  "camera_control_msgs/GrabImagesGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GrabImagesGoal)))
  "Returns string type for a message object of type 'GrabImagesGoal"
  "camera_control_msgs/GrabImagesGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GrabImagesGoal>)))
  "Returns md5sum for a message object of type '<GrabImagesGoal>"
  "c62b1b0356df57d5bde902f61c90337a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GrabImagesGoal)))
  "Returns md5sum for a message object of type 'GrabImagesGoal"
  "c62b1b0356df57d5bde902f61c90337a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GrabImagesGoal>)))
  "Returns full string definition for message of type '<GrabImagesGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%##########################################~%################## GOAL ##################~%##########################################~%~%# Flag which indicates if the exposure times are provided and hence should be~%# set before grabbing~%bool exposure_given~%~%# Only relevant, if exposure_given is true:~%# The list of target exposure times in microseconds.~%# It is possible to grab only one image as well as several images with~%# different exposure times. This values can be overriden from the brightness~%# search, in case that the flag exposure_fixed is not true.~%float32[] exposure_times~%~%# Flag which indicates if the gain is provided and hence should be set before~%# grabbing~%bool gain_given~%~%# Only relevant, if gain_given is true:~%# The target gain in percent of the maximal value the camera supports.~%# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so~%# called 'device specific units'. This value can be overriden from the~%# brightness search, in case that the gain_fixed flag is set to false.~%float32[] gain_values~%~%# Flag which indicates if the gamma value is provided and hence should be set~%# before grabbing~%bool gamma_given~%~%# Only relevant, if gain_given is true:~%# Gamma correction of pixel intensity.~%# Adjusts the brightness of the pixel values output by the camera's sensor~%# to account for a non-linearity in the human perception of brightness or~%# of the display system (such as CRT).~%float32[] gamma_values~%~%# Flag which indicates if the brightness values are provided and hence should~%# be set before grabbing~%bool brightness_given~%~%# Only relevant, if brightness_given is true:~%# The average intensity values of the images. It depends the exposure time~%# as well as the gain setting.~%float32[] brightness_values~%~%# Only relevant, if brightness_given is true:~%# If the camera should try reach the desired brightness, at least one of the~%# following flags MUST be set. If both are set, the interface will use the~%# profile that tries to keep the gain at minimum to reduce white noise.~%# 'exposure_auto' will adapt the exposure time to reach the brightness, wheras~%# 'gain_auto' does so by adapting the gain. If one of these flags is set to~%# false, the connected property will be kept fix.~%# In most of the cases trying to reach a target brightness only by varying the~%# gain and keeping the exposure time fix is not a good approach, because the~%# exposure range is many times higher than the gain range.~%bool exposure_auto~%bool gain_auto~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GrabImagesGoal)))
  "Returns full string definition for message of type 'GrabImagesGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%##########################################~%################## GOAL ##################~%##########################################~%~%# Flag which indicates if the exposure times are provided and hence should be~%# set before grabbing~%bool exposure_given~%~%# Only relevant, if exposure_given is true:~%# The list of target exposure times in microseconds.~%# It is possible to grab only one image as well as several images with~%# different exposure times. This values can be overriden from the brightness~%# search, in case that the flag exposure_fixed is not true.~%float32[] exposure_times~%~%# Flag which indicates if the gain is provided and hence should be set before~%# grabbing~%bool gain_given~%~%# Only relevant, if gain_given is true:~%# The target gain in percent of the maximal value the camera supports.~%# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so~%# called 'device specific units'. This value can be overriden from the~%# brightness search, in case that the gain_fixed flag is set to false.~%float32[] gain_values~%~%# Flag which indicates if the gamma value is provided and hence should be set~%# before grabbing~%bool gamma_given~%~%# Only relevant, if gain_given is true:~%# Gamma correction of pixel intensity.~%# Adjusts the brightness of the pixel values output by the camera's sensor~%# to account for a non-linearity in the human perception of brightness or~%# of the display system (such as CRT).~%float32[] gamma_values~%~%# Flag which indicates if the brightness values are provided and hence should~%# be set before grabbing~%bool brightness_given~%~%# Only relevant, if brightness_given is true:~%# The average intensity values of the images. It depends the exposure time~%# as well as the gain setting.~%float32[] brightness_values~%~%# Only relevant, if brightness_given is true:~%# If the camera should try reach the desired brightness, at least one of the~%# following flags MUST be set. If both are set, the interface will use the~%# profile that tries to keep the gain at minimum to reduce white noise.~%# 'exposure_auto' will adapt the exposure time to reach the brightness, wheras~%# 'gain_auto' does so by adapting the gain. If one of these flags is set to~%# false, the connected property will be kept fix.~%# In most of the cases trying to reach a target brightness only by varying the~%# gain and keeping the exposure time fix is not a good approach, because the~%# exposure range is many times higher than the gain range.~%bool exposure_auto~%bool gain_auto~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GrabImagesGoal>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'exposure_times) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gain_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gamma_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'brightness_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GrabImagesGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'GrabImagesGoal
    (cl:cons ':exposure_given (exposure_given msg))
    (cl:cons ':exposure_times (exposure_times msg))
    (cl:cons ':gain_given (gain_given msg))
    (cl:cons ':gain_values (gain_values msg))
    (cl:cons ':gamma_given (gamma_given msg))
    (cl:cons ':gamma_values (gamma_values msg))
    (cl:cons ':brightness_given (brightness_given msg))
    (cl:cons ':brightness_values (brightness_values msg))
    (cl:cons ':exposure_auto (exposure_auto msg))
    (cl:cons ':gain_auto (gain_auto msg))
))
