; Auto-generated. Do not edit!


(cl:in-package camera_control_msgs-msg)


;//! \htmlinclude GrabHDRImageGoal.msg.html

(cl:defclass <GrabHDRImageGoal> (roslisp-msg-protocol:ros-message)
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
   (exposure_fixed
    :reader exposure_fixed
    :initarg :exposure_fixed
    :type cl:boolean
    :initform cl:nil)
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
   (brightness_continuous
    :reader brightness_continuous
    :initarg :brightness_continuous
    :type cl:boolean
    :initform cl:nil)
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
   (gain_fixed
    :reader gain_fixed
    :initarg :gain_fixed
    :type cl:boolean
    :initform cl:nil)
   (gamma_given
    :reader gamma_given
    :initarg :gamma_given
    :type cl:boolean
    :initform cl:nil)
   (gamma_values
    :reader gamma_values
    :initarg :gamma_values
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GrabHDRImageGoal (<GrabHDRImageGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GrabHDRImageGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GrabHDRImageGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-msg:<GrabHDRImageGoal> is deprecated: use camera_control_msgs-msg:GrabHDRImageGoal instead.")))

(cl:ensure-generic-function 'exposure_given-val :lambda-list '(m))
(cl:defmethod exposure_given-val ((m <GrabHDRImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:exposure_given-val is deprecated.  Use camera_control_msgs-msg:exposure_given instead.")
  (exposure_given m))

(cl:ensure-generic-function 'exposure_times-val :lambda-list '(m))
(cl:defmethod exposure_times-val ((m <GrabHDRImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:exposure_times-val is deprecated.  Use camera_control_msgs-msg:exposure_times instead.")
  (exposure_times m))

(cl:ensure-generic-function 'exposure_fixed-val :lambda-list '(m))
(cl:defmethod exposure_fixed-val ((m <GrabHDRImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:exposure_fixed-val is deprecated.  Use camera_control_msgs-msg:exposure_fixed instead.")
  (exposure_fixed m))

(cl:ensure-generic-function 'brightness_given-val :lambda-list '(m))
(cl:defmethod brightness_given-val ((m <GrabHDRImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:brightness_given-val is deprecated.  Use camera_control_msgs-msg:brightness_given instead.")
  (brightness_given m))

(cl:ensure-generic-function 'brightness_values-val :lambda-list '(m))
(cl:defmethod brightness_values-val ((m <GrabHDRImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:brightness_values-val is deprecated.  Use camera_control_msgs-msg:brightness_values instead.")
  (brightness_values m))

(cl:ensure-generic-function 'brightness_continuous-val :lambda-list '(m))
(cl:defmethod brightness_continuous-val ((m <GrabHDRImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:brightness_continuous-val is deprecated.  Use camera_control_msgs-msg:brightness_continuous instead.")
  (brightness_continuous m))

(cl:ensure-generic-function 'gain_given-val :lambda-list '(m))
(cl:defmethod gain_given-val ((m <GrabHDRImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gain_given-val is deprecated.  Use camera_control_msgs-msg:gain_given instead.")
  (gain_given m))

(cl:ensure-generic-function 'gain_values-val :lambda-list '(m))
(cl:defmethod gain_values-val ((m <GrabHDRImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gain_values-val is deprecated.  Use camera_control_msgs-msg:gain_values instead.")
  (gain_values m))

(cl:ensure-generic-function 'gain_fixed-val :lambda-list '(m))
(cl:defmethod gain_fixed-val ((m <GrabHDRImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gain_fixed-val is deprecated.  Use camera_control_msgs-msg:gain_fixed instead.")
  (gain_fixed m))

(cl:ensure-generic-function 'gamma_given-val :lambda-list '(m))
(cl:defmethod gamma_given-val ((m <GrabHDRImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gamma_given-val is deprecated.  Use camera_control_msgs-msg:gamma_given instead.")
  (gamma_given m))

(cl:ensure-generic-function 'gamma_values-val :lambda-list '(m))
(cl:defmethod gamma_values-val ((m <GrabHDRImageGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-msg:gamma_values-val is deprecated.  Use camera_control_msgs-msg:gamma_values instead.")
  (gamma_values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GrabHDRImageGoal>) ostream)
  "Serializes a message object of type '<GrabHDRImageGoal>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'exposure_fixed) 1 0)) ostream)
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'brightness_continuous) 1 0)) ostream)
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gain_fixed) 1 0)) ostream)
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GrabHDRImageGoal>) istream)
  "Deserializes a message object of type '<GrabHDRImageGoal>"
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
    (cl:setf (cl:slot-value msg 'exposure_fixed) (cl:not (cl:zerop (cl:read-byte istream))))
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
    (cl:setf (cl:slot-value msg 'brightness_continuous) (cl:not (cl:zerop (cl:read-byte istream))))
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
    (cl:setf (cl:slot-value msg 'gain_fixed) (cl:not (cl:zerop (cl:read-byte istream))))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GrabHDRImageGoal>)))
  "Returns string type for a message object of type '<GrabHDRImageGoal>"
  "camera_control_msgs/GrabHDRImageGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GrabHDRImageGoal)))
  "Returns string type for a message object of type 'GrabHDRImageGoal"
  "camera_control_msgs/GrabHDRImageGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GrabHDRImageGoal>)))
  "Returns md5sum for a message object of type '<GrabHDRImageGoal>"
  "4e5897a9b679b570ea1e6aa1d1ce9869")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GrabHDRImageGoal)))
  "Returns md5sum for a message object of type 'GrabHDRImageGoal"
  "4e5897a9b679b570ea1e6aa1d1ce9869")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GrabHDRImageGoal>)))
  "Returns full string definition for message of type '<GrabHDRImageGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%##########################################~%################## GOAL ##################~%##########################################~%~%# Flag which indicates if the exposure times are provided and hence should be~%# set before grabbing~%bool exposure_given~%~%# The list of target exposure time in microseconds.~%# It is possible to grab only one image as well as several images with~%# different exposure times. This values can be overriden from the brightness~%# search, in case that the flag exposure_fixed is not true.~%# The interface will only try to set these values if the exposure_given flag is~%# set to true.~%float32[] exposure_times~%~%# If the exposure_fixed flag is set, the exposure time will stay fix in~%# case of a brightness search. Hence the target brightness will be reached~%# only by varying the gain~%bool exposure_fixed~%~%~%# Flag which indicates if the brightness values are provided and hence should~%# be set before grabbing~%bool brightness_given~%~%# The average intensity values of the images. It depends the exposure time~%# as well as the gain setting. The interface will only try to reach this~%# value if the brightness_given flag is set to true.~%float32[] brightness_values~%~%# The brightness_continuous flag controls the auto brightness function.~%# If it is set to false, the given brightness will only be reached once.~%# Hence changing light conditions lead to changing brightness values.~%# If it is set to true, the given brightness will be reached continuously,~%# trying to adapt to changing light conditions.~%bool brightness_continuous~%~%~%# Flag which indicates if the gain is provided and hence should be set before~%# grabbing~%bool gain_given~%~%# The target gain values in percent of the maximal value the camera supports~%# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so~%# called 'device specific units'. This value can be overriden from the~%# brightness search, in case that the gain_fixed flag is set to false.~%# The interface will only try to reach this values if the gain_given flag is~%# set to true.~%float32[] gain_values~%~%# If the gain_fixed flag is set, the gain value will stay fix in~%# case of a brightness search. Hence the target brightness will be reached~%# only by varying the exposure~%bool gain_fixed~%~%# Flag which indicates if the gamma value is provided and hence should be set~%# before grabbing~%bool gamma_given~%~%# Gamma correction of pixel intensity.~%# Adjusts the brightness of the pixel values output by the camera's sensor~%# to account for a non-linearity in the human perception of brightness or~%# of the display system (such as CRT).~%# The interface will only try to reach this value if the gamma_given flag is~%# set to true.~%float32[] gamma_values~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GrabHDRImageGoal)))
  "Returns full string definition for message of type 'GrabHDRImageGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%##########################################~%################## GOAL ##################~%##########################################~%~%# Flag which indicates if the exposure times are provided and hence should be~%# set before grabbing~%bool exposure_given~%~%# The list of target exposure time in microseconds.~%# It is possible to grab only one image as well as several images with~%# different exposure times. This values can be overriden from the brightness~%# search, in case that the flag exposure_fixed is not true.~%# The interface will only try to set these values if the exposure_given flag is~%# set to true.~%float32[] exposure_times~%~%# If the exposure_fixed flag is set, the exposure time will stay fix in~%# case of a brightness search. Hence the target brightness will be reached~%# only by varying the gain~%bool exposure_fixed~%~%~%# Flag which indicates if the brightness values are provided and hence should~%# be set before grabbing~%bool brightness_given~%~%# The average intensity values of the images. It depends the exposure time~%# as well as the gain setting. The interface will only try to reach this~%# value if the brightness_given flag is set to true.~%float32[] brightness_values~%~%# The brightness_continuous flag controls the auto brightness function.~%# If it is set to false, the given brightness will only be reached once.~%# Hence changing light conditions lead to changing brightness values.~%# If it is set to true, the given brightness will be reached continuously,~%# trying to adapt to changing light conditions.~%bool brightness_continuous~%~%~%# Flag which indicates if the gain is provided and hence should be set before~%# grabbing~%bool gain_given~%~%# The target gain values in percent of the maximal value the camera supports~%# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so~%# called 'device specific units'. This value can be overriden from the~%# brightness search, in case that the gain_fixed flag is set to false.~%# The interface will only try to reach this values if the gain_given flag is~%# set to true.~%float32[] gain_values~%~%# If the gain_fixed flag is set, the gain value will stay fix in~%# case of a brightness search. Hence the target brightness will be reached~%# only by varying the exposure~%bool gain_fixed~%~%# Flag which indicates if the gamma value is provided and hence should be set~%# before grabbing~%bool gamma_given~%~%# Gamma correction of pixel intensity.~%# Adjusts the brightness of the pixel values output by the camera's sensor~%# to account for a non-linearity in the human perception of brightness or~%# of the display system (such as CRT).~%# The interface will only try to reach this value if the gamma_given flag is~%# set to true.~%float32[] gamma_values~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GrabHDRImageGoal>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'exposure_times) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'brightness_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gain_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gamma_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GrabHDRImageGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'GrabHDRImageGoal
    (cl:cons ':exposure_given (exposure_given msg))
    (cl:cons ':exposure_times (exposure_times msg))
    (cl:cons ':exposure_fixed (exposure_fixed msg))
    (cl:cons ':brightness_given (brightness_given msg))
    (cl:cons ':brightness_values (brightness_values msg))
    (cl:cons ':brightness_continuous (brightness_continuous msg))
    (cl:cons ':gain_given (gain_given msg))
    (cl:cons ':gain_values (gain_values msg))
    (cl:cons ':gain_fixed (gain_fixed msg))
    (cl:cons ':gamma_given (gamma_given msg))
    (cl:cons ':gamma_values (gamma_values msg))
))
