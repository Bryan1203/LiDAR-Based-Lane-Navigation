;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::SetBrightness)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'SetBrightness (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::SETBRIGHTNESS")
  (make-package "CAMERA_CONTROL_MSGS::SETBRIGHTNESS"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETBRIGHTNESSREQUEST")
  (make-package "CAMERA_CONTROL_MSGS::SETBRIGHTNESSREQUEST"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETBRIGHTNESSRESPONSE")
  (make-package "CAMERA_CONTROL_MSGS::SETBRIGHTNESSRESPONSE"))

(in-package "ROS")





(defclass camera_control_msgs::SetBrightnessRequest
  :super ros::object
  :slots (_target_brightness _brightness_continuous _exposure_auto _gain_auto ))

(defmethod camera_control_msgs::SetBrightnessRequest
  (:init
   (&key
    ((:target_brightness __target_brightness) 0)
    ((:brightness_continuous __brightness_continuous) nil)
    ((:exposure_auto __exposure_auto) nil)
    ((:gain_auto __gain_auto) nil)
    )
   (send-super :init)
   (setq _target_brightness (round __target_brightness))
   (setq _brightness_continuous __brightness_continuous)
   (setq _exposure_auto __exposure_auto)
   (setq _gain_auto __gain_auto)
   self)
  (:target_brightness
   (&optional __target_brightness)
   (if __target_brightness (setq _target_brightness __target_brightness)) _target_brightness)
  (:brightness_continuous
   (&optional (__brightness_continuous :null))
   (if (not (eq __brightness_continuous :null)) (setq _brightness_continuous __brightness_continuous)) _brightness_continuous)
  (:exposure_auto
   (&optional (__exposure_auto :null))
   (if (not (eq __exposure_auto :null)) (setq _exposure_auto __exposure_auto)) _exposure_auto)
  (:gain_auto
   (&optional (__gain_auto :null))
   (if (not (eq __gain_auto :null)) (setq _gain_auto __gain_auto)) _gain_auto)
  (:serialization-length
   ()
   (+
    ;; int32 _target_brightness
    4
    ;; bool _brightness_continuous
    1
    ;; bool _exposure_auto
    1
    ;; bool _gain_auto
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _target_brightness
       (write-long _target_brightness s)
     ;; bool _brightness_continuous
       (if _brightness_continuous (write-byte -1 s) (write-byte 0 s))
     ;; bool _exposure_auto
       (if _exposure_auto (write-byte -1 s) (write-byte 0 s))
     ;; bool _gain_auto
       (if _gain_auto (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _target_brightness
     (setq _target_brightness (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _brightness_continuous
     (setq _brightness_continuous (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _exposure_auto
     (setq _exposure_auto (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gain_auto
     (setq _gain_auto (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_control_msgs::SetBrightnessResponse
  :super ros::object
  :slots (_reached_brightness _reached_exposure_time _reached_gain_value _success ))

(defmethod camera_control_msgs::SetBrightnessResponse
  (:init
   (&key
    ((:reached_brightness __reached_brightness) 0)
    ((:reached_exposure_time __reached_exposure_time) 0.0)
    ((:reached_gain_value __reached_gain_value) 0.0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _reached_brightness (round __reached_brightness))
   (setq _reached_exposure_time (float __reached_exposure_time))
   (setq _reached_gain_value (float __reached_gain_value))
   (setq _success __success)
   self)
  (:reached_brightness
   (&optional __reached_brightness)
   (if __reached_brightness (setq _reached_brightness __reached_brightness)) _reached_brightness)
  (:reached_exposure_time
   (&optional __reached_exposure_time)
   (if __reached_exposure_time (setq _reached_exposure_time __reached_exposure_time)) _reached_exposure_time)
  (:reached_gain_value
   (&optional __reached_gain_value)
   (if __reached_gain_value (setq _reached_gain_value __reached_gain_value)) _reached_gain_value)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; int32 _reached_brightness
    4
    ;; float32 _reached_exposure_time
    4
    ;; float32 _reached_gain_value
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _reached_brightness
       (write-long _reached_brightness s)
     ;; float32 _reached_exposure_time
       (sys::poke _reached_exposure_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _reached_gain_value
       (sys::poke _reached_gain_value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _reached_brightness
     (setq _reached_brightness (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _reached_exposure_time
     (setq _reached_exposure_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _reached_gain_value
     (setq _reached_gain_value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_control_msgs::SetBrightness
  :super ros::object
  :slots ())

(setf (get camera_control_msgs::SetBrightness :md5sum-) "36481c45e92096457660dc69b68eb6b1")
(setf (get camera_control_msgs::SetBrightness :datatype-) "camera_control_msgs/SetBrightness")
(setf (get camera_control_msgs::SetBrightness :request) camera_control_msgs::SetBrightnessRequest)
(setf (get camera_control_msgs::SetBrightness :response) camera_control_msgs::SetBrightnessResponse)

(defmethod camera_control_msgs::SetBrightnessRequest
  (:response () (instance camera_control_msgs::SetBrightnessResponse :init)))

(setf (get camera_control_msgs::SetBrightnessRequest :md5sum-) "36481c45e92096457660dc69b68eb6b1")
(setf (get camera_control_msgs::SetBrightnessRequest :datatype-) "camera_control_msgs/SetBrightnessRequest")
(setf (get camera_control_msgs::SetBrightnessRequest :definition-)
      "# The target brightness, which is average intensity values of the images.
# It depends the exposure time as well as the gain setting.
int32 target_brightness

# The brightness_continuous flag controls the auto brightness function.
# If it is set to false, the given brightness will only be reached once.
# Hence changing light conditions lead to changing brightness values.
# If it is set to true, the given brightness will be reached continuously,
# trying to adapt to changing light conditions. The 'brightness_contunuous'
# mode is is only possible for values in the possible auto range of the arena
# API which is e.g. [50 - 205] for acA2500-14um and acA1920-40gm
bool brightness_continuous

# If the camera should try reach or keep the desired brightness, hence adapting
# to changing light conditions, at least one of the following flags MUST be set.
# If both are set, the interface will use the profile that tries to keep the
# gain at minimum to reduce white noise.
# 'exposure_auto' will adapt the exposure time to reach the brightness, wheras
# 'gain_auto' does so by adapting the gain.
bool exposure_auto
bool gain_auto

---

# Exact match can not always be reached
int32 reached_brightness
float32 reached_exposure_time
float32 reached_gain_value

bool success

")

(setf (get camera_control_msgs::SetBrightnessResponse :md5sum-) "36481c45e92096457660dc69b68eb6b1")
(setf (get camera_control_msgs::SetBrightnessResponse :datatype-) "camera_control_msgs/SetBrightnessResponse")
(setf (get camera_control_msgs::SetBrightnessResponse :definition-)
      "# The target brightness, which is average intensity values of the images.
# It depends the exposure time as well as the gain setting.
int32 target_brightness

# The brightness_continuous flag controls the auto brightness function.
# If it is set to false, the given brightness will only be reached once.
# Hence changing light conditions lead to changing brightness values.
# If it is set to true, the given brightness will be reached continuously,
# trying to adapt to changing light conditions. The 'brightness_contunuous'
# mode is is only possible for values in the possible auto range of the arena
# API which is e.g. [50 - 205] for acA2500-14um and acA1920-40gm
bool brightness_continuous

# If the camera should try reach or keep the desired brightness, hence adapting
# to changing light conditions, at least one of the following flags MUST be set.
# If both are set, the interface will use the profile that tries to keep the
# gain at minimum to reduce white noise.
# 'exposure_auto' will adapt the exposure time to reach the brightness, wheras
# 'gain_auto' does so by adapting the gain.
bool exposure_auto
bool gain_auto

---

# Exact match can not always be reached
int32 reached_brightness
float32 reached_exposure_time
float32 reached_gain_value

bool success

")



(provide :camera_control_msgs/SetBrightness "36481c45e92096457660dc69b68eb6b1")


