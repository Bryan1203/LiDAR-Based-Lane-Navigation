;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::SetGain)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'SetGain (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::SETGAIN")
  (make-package "CAMERA_CONTROL_MSGS::SETGAIN"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETGAINREQUEST")
  (make-package "CAMERA_CONTROL_MSGS::SETGAINREQUEST"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETGAINRESPONSE")
  (make-package "CAMERA_CONTROL_MSGS::SETGAINRESPONSE"))

(in-package "ROS")





(defclass camera_control_msgs::SetGainRequest
  :super ros::object
  :slots (_target_gain ))

(defmethod camera_control_msgs::SetGainRequest
  (:init
   (&key
    ((:target_gain __target_gain) 0.0)
    )
   (send-super :init)
   (setq _target_gain (float __target_gain))
   self)
  (:target_gain
   (&optional __target_gain)
   (if __target_gain (setq _target_gain __target_gain)) _target_gain)
  (:serialization-length
   ()
   (+
    ;; float32 _target_gain
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _target_gain
       (sys::poke _target_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _target_gain
     (setq _target_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass camera_control_msgs::SetGainResponse
  :super ros::object
  :slots (_reached_gain _success ))

(defmethod camera_control_msgs::SetGainResponse
  (:init
   (&key
    ((:reached_gain __reached_gain) 0.0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _reached_gain (float __reached_gain))
   (setq _success __success)
   self)
  (:reached_gain
   (&optional __reached_gain)
   (if __reached_gain (setq _reached_gain __reached_gain)) _reached_gain)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float32 _reached_gain
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _reached_gain
       (sys::poke _reached_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _reached_gain
     (setq _reached_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_control_msgs::SetGain
  :super ros::object
  :slots ())

(setf (get camera_control_msgs::SetGain :md5sum-) "20482afb1d808bae25db22114b0a747f")
(setf (get camera_control_msgs::SetGain :datatype-) "camera_control_msgs/SetGain")
(setf (get camera_control_msgs::SetGain :request) camera_control_msgs::SetGainRequest)
(setf (get camera_control_msgs::SetGain :response) camera_control_msgs::SetGainResponse)

(defmethod camera_control_msgs::SetGainRequest
  (:response () (instance camera_control_msgs::SetGainResponse :init)))

(setf (get camera_control_msgs::SetGainRequest :md5sum-) "20482afb1d808bae25db22114b0a747f")
(setf (get camera_control_msgs::SetGainRequest :datatype-) "camera_control_msgs/SetGainRequest")
(setf (get camera_control_msgs::SetGainRequest :definition-)
      "# The target gain in percent of the maximal value the camera supports.
# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
# called 'device specific units'.
float32 target_gain
---
# Exact match can not always be reached
float32 reached_gain
bool success

")

(setf (get camera_control_msgs::SetGainResponse :md5sum-) "20482afb1d808bae25db22114b0a747f")
(setf (get camera_control_msgs::SetGainResponse :datatype-) "camera_control_msgs/SetGainResponse")
(setf (get camera_control_msgs::SetGainResponse :definition-)
      "# The target gain in percent of the maximal value the camera supports.
# For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
# called 'device specific units'.
float32 target_gain
---
# Exact match can not always be reached
float32 reached_gain
bool success

")



(provide :camera_control_msgs/SetGain "20482afb1d808bae25db22114b0a747f")


