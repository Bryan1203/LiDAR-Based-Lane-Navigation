;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::GrabAndSaveImageGoal)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'GrabAndSaveImageGoal (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::GRABANDSAVEIMAGEGOAL")
  (make-package "CAMERA_CONTROL_MSGS::GRABANDSAVEIMAGEGOAL"))

(in-package "ROS")
;;//! \htmlinclude GrabAndSaveImageGoal.msg.html


(defclass camera_control_msgs::GrabAndSaveImageGoal
  :super ros::object
  :slots (_exposure_given _exposure_time _gain_given _gain_value _gamma_given _gamma_value _brightness_given _brightness_value _exposure_auto _gain_auto _img_storage_path_and_name ))

(defmethod camera_control_msgs::GrabAndSaveImageGoal
  (:init
   (&key
    ((:exposure_given __exposure_given) nil)
    ((:exposure_time __exposure_time) 0.0)
    ((:gain_given __gain_given) nil)
    ((:gain_value __gain_value) 0.0)
    ((:gamma_given __gamma_given) nil)
    ((:gamma_value __gamma_value) 0.0)
    ((:brightness_given __brightness_given) nil)
    ((:brightness_value __brightness_value) 0.0)
    ((:exposure_auto __exposure_auto) nil)
    ((:gain_auto __gain_auto) nil)
    ((:img_storage_path_and_name __img_storage_path_and_name) "")
    )
   (send-super :init)
   (setq _exposure_given __exposure_given)
   (setq _exposure_time (float __exposure_time))
   (setq _gain_given __gain_given)
   (setq _gain_value (float __gain_value))
   (setq _gamma_given __gamma_given)
   (setq _gamma_value (float __gamma_value))
   (setq _brightness_given __brightness_given)
   (setq _brightness_value (float __brightness_value))
   (setq _exposure_auto __exposure_auto)
   (setq _gain_auto __gain_auto)
   (setq _img_storage_path_and_name (string __img_storage_path_and_name))
   self)
  (:exposure_given
   (&optional (__exposure_given :null))
   (if (not (eq __exposure_given :null)) (setq _exposure_given __exposure_given)) _exposure_given)
  (:exposure_time
   (&optional __exposure_time)
   (if __exposure_time (setq _exposure_time __exposure_time)) _exposure_time)
  (:gain_given
   (&optional (__gain_given :null))
   (if (not (eq __gain_given :null)) (setq _gain_given __gain_given)) _gain_given)
  (:gain_value
   (&optional __gain_value)
   (if __gain_value (setq _gain_value __gain_value)) _gain_value)
  (:gamma_given
   (&optional (__gamma_given :null))
   (if (not (eq __gamma_given :null)) (setq _gamma_given __gamma_given)) _gamma_given)
  (:gamma_value
   (&optional __gamma_value)
   (if __gamma_value (setq _gamma_value __gamma_value)) _gamma_value)
  (:brightness_given
   (&optional (__brightness_given :null))
   (if (not (eq __brightness_given :null)) (setq _brightness_given __brightness_given)) _brightness_given)
  (:brightness_value
   (&optional __brightness_value)
   (if __brightness_value (setq _brightness_value __brightness_value)) _brightness_value)
  (:exposure_auto
   (&optional (__exposure_auto :null))
   (if (not (eq __exposure_auto :null)) (setq _exposure_auto __exposure_auto)) _exposure_auto)
  (:gain_auto
   (&optional (__gain_auto :null))
   (if (not (eq __gain_auto :null)) (setq _gain_auto __gain_auto)) _gain_auto)
  (:img_storage_path_and_name
   (&optional __img_storage_path_and_name)
   (if __img_storage_path_and_name (setq _img_storage_path_and_name __img_storage_path_and_name)) _img_storage_path_and_name)
  (:serialization-length
   ()
   (+
    ;; bool _exposure_given
    1
    ;; float32 _exposure_time
    4
    ;; bool _gain_given
    1
    ;; float32 _gain_value
    4
    ;; bool _gamma_given
    1
    ;; float32 _gamma_value
    4
    ;; bool _brightness_given
    1
    ;; float32 _brightness_value
    4
    ;; bool _exposure_auto
    1
    ;; bool _gain_auto
    1
    ;; string _img_storage_path_and_name
    4 (length _img_storage_path_and_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _exposure_given
       (if _exposure_given (write-byte -1 s) (write-byte 0 s))
     ;; float32 _exposure_time
       (sys::poke _exposure_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _gain_given
       (if _gain_given (write-byte -1 s) (write-byte 0 s))
     ;; float32 _gain_value
       (sys::poke _gain_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _gamma_given
       (if _gamma_given (write-byte -1 s) (write-byte 0 s))
     ;; float32 _gamma_value
       (sys::poke _gamma_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _brightness_given
       (if _brightness_given (write-byte -1 s) (write-byte 0 s))
     ;; float32 _brightness_value
       (sys::poke _brightness_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _exposure_auto
       (if _exposure_auto (write-byte -1 s) (write-byte 0 s))
     ;; bool _gain_auto
       (if _gain_auto (write-byte -1 s) (write-byte 0 s))
     ;; string _img_storage_path_and_name
       (write-long (length _img_storage_path_and_name) s) (princ _img_storage_path_and_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _exposure_given
     (setq _exposure_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _exposure_time
     (setq _exposure_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _gain_given
     (setq _gain_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _gain_value
     (setq _gain_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _gamma_given
     (setq _gamma_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _gamma_value
     (setq _gamma_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _brightness_given
     (setq _brightness_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _brightness_value
     (setq _brightness_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _exposure_auto
     (setq _exposure_auto (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gain_auto
     (setq _gain_auto (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _img_storage_path_and_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _img_storage_path_and_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get camera_control_msgs::GrabAndSaveImageGoal :md5sum-) "c66e7613728d45278cbdc7c5a89f6e08")
(setf (get camera_control_msgs::GrabAndSaveImageGoal :datatype-) "camera_control_msgs/GrabAndSaveImageGoal")
(setf (get camera_control_msgs::GrabAndSaveImageGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# The 'GrabAndSaveImageAction' has a similar goal as the 'GrabImagesAction' in
# case of only grabbing one image. It additionally contains a string describing
# the full storage path of the image to be captured.
# It wont provide the grabbed image via the result topic. Instead it only
# returns a flag indicating the success.

##########################################
################## GOAL ##################
##########################################

# Flag which indicates if the exposure time is provided and hence should be
# set before grabbing
bool exposure_given

# Only relevant, if exposure_given is true:
# The target exposure time in microseconds. This values can be overriden from
# the brightness search, in case that the flag exposure_fixed is not true.
float32 exposure_time

# Flag which indicates if the gain is provided and hence should be set before
# grabbing
bool gain_given

# Only relevant, if gain_given is true:
# The target gain in percent of the maximal value the camera supports.
# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
# called 'device specific units'. This value can be overriden from the
# brightness search, in case that the gain_fixed flag is set to false.
float32 gain_value

# Flag which indicates if the gamma value is provided and hence should be set
# before grabbing
bool gamma_given

# Only relevant, if gain_given is true:
# Gamma correction of pixel intensity.
# Adjusts the brightness of the pixel values output by the camera's sensor
# to account for a non-linearity in the human perception of brightness or
# of the display system (such as CRT).
float32 gamma_value

# Flag which indicates if the brightness value is provided and hence should
# be set before grabbing
bool brightness_given

# Only relevant, if brightness_given is true:
# The average intensity value of the resulting image. It depends the exposure
# time as well as the gain setting.
float32 brightness_value

# Only relevant, if brightness_given is true:
# If the camera should try reach the desired brightness, at least one of the
# following flags MUST be set. If both are set, the interface will use the
# profile that tries to keep the gain at minimum to reduce white noise.
# 'exposure_auto' will adapt the exposure time to reach the brightness, wheras
# 'gain_auto' does so by adapting the gain. If one of these flags is set to
# false, the connected property will be kept fix.
# In most of the cases trying to reach a target brightness only by varying the
# gain and keeping the exposure time fix is not a good approach, because the
# exposure range is many times higher than the gain range.
bool exposure_auto
bool gain_auto

# File path and image name (including file-extension) to store the grabbed
# image
string img_storage_path_and_name

")



(provide :camera_control_msgs/GrabAndSaveImageGoal "c66e7613728d45278cbdc7c5a89f6e08")


