;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::GrabImagesGoal)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'GrabImagesGoal (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::GRABIMAGESGOAL")
  (make-package "CAMERA_CONTROL_MSGS::GRABIMAGESGOAL"))

(in-package "ROS")
;;//! \htmlinclude GrabImagesGoal.msg.html


(defclass camera_control_msgs::GrabImagesGoal
  :super ros::object
  :slots (_exposure_given _exposure_times _gain_given _gain_values _gamma_given _gamma_values _brightness_given _brightness_values _exposure_auto _gain_auto ))

(defmethod camera_control_msgs::GrabImagesGoal
  (:init
   (&key
    ((:exposure_given __exposure_given) nil)
    ((:exposure_times __exposure_times) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:gain_given __gain_given) nil)
    ((:gain_values __gain_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:gamma_given __gamma_given) nil)
    ((:gamma_values __gamma_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:brightness_given __brightness_given) nil)
    ((:brightness_values __brightness_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:exposure_auto __exposure_auto) nil)
    ((:gain_auto __gain_auto) nil)
    )
   (send-super :init)
   (setq _exposure_given __exposure_given)
   (setq _exposure_times __exposure_times)
   (setq _gain_given __gain_given)
   (setq _gain_values __gain_values)
   (setq _gamma_given __gamma_given)
   (setq _gamma_values __gamma_values)
   (setq _brightness_given __brightness_given)
   (setq _brightness_values __brightness_values)
   (setq _exposure_auto __exposure_auto)
   (setq _gain_auto __gain_auto)
   self)
  (:exposure_given
   (&optional (__exposure_given :null))
   (if (not (eq __exposure_given :null)) (setq _exposure_given __exposure_given)) _exposure_given)
  (:exposure_times
   (&optional __exposure_times)
   (if __exposure_times (setq _exposure_times __exposure_times)) _exposure_times)
  (:gain_given
   (&optional (__gain_given :null))
   (if (not (eq __gain_given :null)) (setq _gain_given __gain_given)) _gain_given)
  (:gain_values
   (&optional __gain_values)
   (if __gain_values (setq _gain_values __gain_values)) _gain_values)
  (:gamma_given
   (&optional (__gamma_given :null))
   (if (not (eq __gamma_given :null)) (setq _gamma_given __gamma_given)) _gamma_given)
  (:gamma_values
   (&optional __gamma_values)
   (if __gamma_values (setq _gamma_values __gamma_values)) _gamma_values)
  (:brightness_given
   (&optional (__brightness_given :null))
   (if (not (eq __brightness_given :null)) (setq _brightness_given __brightness_given)) _brightness_given)
  (:brightness_values
   (&optional __brightness_values)
   (if __brightness_values (setq _brightness_values __brightness_values)) _brightness_values)
  (:exposure_auto
   (&optional (__exposure_auto :null))
   (if (not (eq __exposure_auto :null)) (setq _exposure_auto __exposure_auto)) _exposure_auto)
  (:gain_auto
   (&optional (__gain_auto :null))
   (if (not (eq __gain_auto :null)) (setq _gain_auto __gain_auto)) _gain_auto)
  (:serialization-length
   ()
   (+
    ;; bool _exposure_given
    1
    ;; float32[] _exposure_times
    (* 4    (length _exposure_times)) 4
    ;; bool _gain_given
    1
    ;; float32[] _gain_values
    (* 4    (length _gain_values)) 4
    ;; bool _gamma_given
    1
    ;; float32[] _gamma_values
    (* 4    (length _gamma_values)) 4
    ;; bool _brightness_given
    1
    ;; float32[] _brightness_values
    (* 4    (length _brightness_values)) 4
    ;; bool _exposure_auto
    1
    ;; bool _gain_auto
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _exposure_given
       (if _exposure_given (write-byte -1 s) (write-byte 0 s))
     ;; float32[] _exposure_times
     (write-long (length _exposure_times) s)
     (dotimes (i (length _exposure_times))
       (sys::poke (elt _exposure_times i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _gain_given
       (if _gain_given (write-byte -1 s) (write-byte 0 s))
     ;; float32[] _gain_values
     (write-long (length _gain_values) s)
     (dotimes (i (length _gain_values))
       (sys::poke (elt _gain_values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _gamma_given
       (if _gamma_given (write-byte -1 s) (write-byte 0 s))
     ;; float32[] _gamma_values
     (write-long (length _gamma_values) s)
     (dotimes (i (length _gamma_values))
       (sys::poke (elt _gamma_values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _brightness_given
       (if _brightness_given (write-byte -1 s) (write-byte 0 s))
     ;; float32[] _brightness_values
     (write-long (length _brightness_values) s)
     (dotimes (i (length _brightness_values))
       (sys::poke (elt _brightness_values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _exposure_auto
       (if _exposure_auto (write-byte -1 s) (write-byte 0 s))
     ;; bool _gain_auto
       (if _gain_auto (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _exposure_given
     (setq _exposure_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32[] _exposure_times
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _exposure_times (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _exposure_times i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _gain_given
     (setq _gain_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32[] _gain_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gain_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _gain_values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _gamma_given
     (setq _gamma_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32[] _gamma_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gamma_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _gamma_values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _brightness_given
     (setq _brightness_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32[] _brightness_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _brightness_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _brightness_values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _exposure_auto
     (setq _exposure_auto (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gain_auto
     (setq _gain_auto (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get camera_control_msgs::GrabImagesGoal :md5sum-) "c62b1b0356df57d5bde902f61c90337a")
(setf (get camera_control_msgs::GrabImagesGoal :datatype-) "camera_control_msgs/GrabImagesGoal")
(setf (get camera_control_msgs::GrabImagesGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
##########################################
################## GOAL ##################
##########################################

# Flag which indicates if the exposure times are provided and hence should be
# set before grabbing
bool exposure_given

# Only relevant, if exposure_given is true:
# The list of target exposure times in microseconds.
# It is possible to grab only one image as well as several images with
# different exposure times. This values can be overriden from the brightness
# search, in case that the flag exposure_fixed is not true.
float32[] exposure_times

# Flag which indicates if the gain is provided and hence should be set before
# grabbing
bool gain_given

# Only relevant, if gain_given is true:
# The target gain in percent of the maximal value the camera supports.
# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
# called 'device specific units'. This value can be overriden from the
# brightness search, in case that the gain_fixed flag is set to false.
float32[] gain_values

# Flag which indicates if the gamma value is provided and hence should be set
# before grabbing
bool gamma_given

# Only relevant, if gain_given is true:
# Gamma correction of pixel intensity.
# Adjusts the brightness of the pixel values output by the camera's sensor
# to account for a non-linearity in the human perception of brightness or
# of the display system (such as CRT).
float32[] gamma_values

# Flag which indicates if the brightness values are provided and hence should
# be set before grabbing
bool brightness_given

# Only relevant, if brightness_given is true:
# The average intensity values of the images. It depends the exposure time
# as well as the gain setting.
float32[] brightness_values

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


")



(provide :camera_control_msgs/GrabImagesGoal "c62b1b0356df57d5bde902f61c90337a")


