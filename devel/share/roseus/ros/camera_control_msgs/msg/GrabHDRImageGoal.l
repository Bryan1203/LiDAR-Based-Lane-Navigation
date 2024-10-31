;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::GrabHDRImageGoal)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'GrabHDRImageGoal (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::GRABHDRIMAGEGOAL")
  (make-package "CAMERA_CONTROL_MSGS::GRABHDRIMAGEGOAL"))

(in-package "ROS")
;;//! \htmlinclude GrabHDRImageGoal.msg.html


(defclass camera_control_msgs::GrabHDRImageGoal
  :super ros::object
  :slots (_exposure_given _exposure_times _exposure_fixed _brightness_given _brightness_values _brightness_continuous _gain_given _gain_values _gain_fixed _gamma_given _gamma_values ))

(defmethod camera_control_msgs::GrabHDRImageGoal
  (:init
   (&key
    ((:exposure_given __exposure_given) nil)
    ((:exposure_times __exposure_times) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:exposure_fixed __exposure_fixed) nil)
    ((:brightness_given __brightness_given) nil)
    ((:brightness_values __brightness_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:brightness_continuous __brightness_continuous) nil)
    ((:gain_given __gain_given) nil)
    ((:gain_values __gain_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:gain_fixed __gain_fixed) nil)
    ((:gamma_given __gamma_given) nil)
    ((:gamma_values __gamma_values) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _exposure_given __exposure_given)
   (setq _exposure_times __exposure_times)
   (setq _exposure_fixed __exposure_fixed)
   (setq _brightness_given __brightness_given)
   (setq _brightness_values __brightness_values)
   (setq _brightness_continuous __brightness_continuous)
   (setq _gain_given __gain_given)
   (setq _gain_values __gain_values)
   (setq _gain_fixed __gain_fixed)
   (setq _gamma_given __gamma_given)
   (setq _gamma_values __gamma_values)
   self)
  (:exposure_given
   (&optional (__exposure_given :null))
   (if (not (eq __exposure_given :null)) (setq _exposure_given __exposure_given)) _exposure_given)
  (:exposure_times
   (&optional __exposure_times)
   (if __exposure_times (setq _exposure_times __exposure_times)) _exposure_times)
  (:exposure_fixed
   (&optional (__exposure_fixed :null))
   (if (not (eq __exposure_fixed :null)) (setq _exposure_fixed __exposure_fixed)) _exposure_fixed)
  (:brightness_given
   (&optional (__brightness_given :null))
   (if (not (eq __brightness_given :null)) (setq _brightness_given __brightness_given)) _brightness_given)
  (:brightness_values
   (&optional __brightness_values)
   (if __brightness_values (setq _brightness_values __brightness_values)) _brightness_values)
  (:brightness_continuous
   (&optional (__brightness_continuous :null))
   (if (not (eq __brightness_continuous :null)) (setq _brightness_continuous __brightness_continuous)) _brightness_continuous)
  (:gain_given
   (&optional (__gain_given :null))
   (if (not (eq __gain_given :null)) (setq _gain_given __gain_given)) _gain_given)
  (:gain_values
   (&optional __gain_values)
   (if __gain_values (setq _gain_values __gain_values)) _gain_values)
  (:gain_fixed
   (&optional (__gain_fixed :null))
   (if (not (eq __gain_fixed :null)) (setq _gain_fixed __gain_fixed)) _gain_fixed)
  (:gamma_given
   (&optional (__gamma_given :null))
   (if (not (eq __gamma_given :null)) (setq _gamma_given __gamma_given)) _gamma_given)
  (:gamma_values
   (&optional __gamma_values)
   (if __gamma_values (setq _gamma_values __gamma_values)) _gamma_values)
  (:serialization-length
   ()
   (+
    ;; bool _exposure_given
    1
    ;; float32[] _exposure_times
    (* 4    (length _exposure_times)) 4
    ;; bool _exposure_fixed
    1
    ;; bool _brightness_given
    1
    ;; float32[] _brightness_values
    (* 4    (length _brightness_values)) 4
    ;; bool _brightness_continuous
    1
    ;; bool _gain_given
    1
    ;; float32[] _gain_values
    (* 4    (length _gain_values)) 4
    ;; bool _gain_fixed
    1
    ;; bool _gamma_given
    1
    ;; float32[] _gamma_values
    (* 4    (length _gamma_values)) 4
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
     ;; bool _exposure_fixed
       (if _exposure_fixed (write-byte -1 s) (write-byte 0 s))
     ;; bool _brightness_given
       (if _brightness_given (write-byte -1 s) (write-byte 0 s))
     ;; float32[] _brightness_values
     (write-long (length _brightness_values) s)
     (dotimes (i (length _brightness_values))
       (sys::poke (elt _brightness_values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _brightness_continuous
       (if _brightness_continuous (write-byte -1 s) (write-byte 0 s))
     ;; bool _gain_given
       (if _gain_given (write-byte -1 s) (write-byte 0 s))
     ;; float32[] _gain_values
     (write-long (length _gain_values) s)
     (dotimes (i (length _gain_values))
       (sys::poke (elt _gain_values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _gain_fixed
       (if _gain_fixed (write-byte -1 s) (write-byte 0 s))
     ;; bool _gamma_given
       (if _gamma_given (write-byte -1 s) (write-byte 0 s))
     ;; float32[] _gamma_values
     (write-long (length _gamma_values) s)
     (dotimes (i (length _gamma_values))
       (sys::poke (elt _gamma_values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
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
   ;; bool _exposure_fixed
     (setq _exposure_fixed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _brightness_given
     (setq _brightness_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32[] _brightness_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _brightness_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _brightness_values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _brightness_continuous
     (setq _brightness_continuous (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gain_given
     (setq _gain_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32[] _gain_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gain_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _gain_values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _gain_fixed
     (setq _gain_fixed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gamma_given
     (setq _gamma_given (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32[] _gamma_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gamma_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _gamma_values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get camera_control_msgs::GrabHDRImageGoal :md5sum-) "4e5897a9b679b570ea1e6aa1d1ce9869")
(setf (get camera_control_msgs::GrabHDRImageGoal :datatype-) "camera_control_msgs/GrabHDRImageGoal")
(setf (get camera_control_msgs::GrabHDRImageGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
##########################################
################## GOAL ##################
##########################################

# Flag which indicates if the exposure times are provided and hence should be
# set before grabbing
bool exposure_given

# The list of target exposure time in microseconds.
# It is possible to grab only one image as well as several images with
# different exposure times. This values can be overriden from the brightness
# search, in case that the flag exposure_fixed is not true.
# The interface will only try to set these values if the exposure_given flag is
# set to true.
float32[] exposure_times

# If the exposure_fixed flag is set, the exposure time will stay fix in
# case of a brightness search. Hence the target brightness will be reached
# only by varying the gain
bool exposure_fixed


# Flag which indicates if the brightness values are provided and hence should
# be set before grabbing
bool brightness_given

# The average intensity values of the images. It depends the exposure time
# as well as the gain setting. The interface will only try to reach this
# value if the brightness_given flag is set to true.
float32[] brightness_values

# The brightness_continuous flag controls the auto brightness function.
# If it is set to false, the given brightness will only be reached once.
# Hence changing light conditions lead to changing brightness values.
# If it is set to true, the given brightness will be reached continuously,
# trying to adapt to changing light conditions.
bool brightness_continuous


# Flag which indicates if the gain is provided and hence should be set before
# grabbing
bool gain_given

# The target gain values in percent of the maximal value the camera supports
# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
# called 'device specific units'. This value can be overriden from the
# brightness search, in case that the gain_fixed flag is set to false.
# The interface will only try to reach this values if the gain_given flag is
# set to true.
float32[] gain_values

# If the gain_fixed flag is set, the gain value will stay fix in
# case of a brightness search. Hence the target brightness will be reached
# only by varying the exposure
bool gain_fixed

# Flag which indicates if the gamma value is provided and hence should be set
# before grabbing
bool gamma_given

# Gamma correction of pixel intensity.
# Adjusts the brightness of the pixel values output by the camera's sensor
# to account for a non-linearity in the human perception of brightness or
# of the display system (such as CRT).
# The interface will only try to reach this value if the gamma_given flag is
# set to true.
float32[] gamma_values


")



(provide :camera_control_msgs/GrabHDRImageGoal "4e5897a9b679b570ea1e6aa1d1ce9869")


