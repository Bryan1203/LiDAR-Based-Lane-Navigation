;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::SetGamma)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'SetGamma (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::SETGAMMA")
  (make-package "CAMERA_CONTROL_MSGS::SETGAMMA"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETGAMMAREQUEST")
  (make-package "CAMERA_CONTROL_MSGS::SETGAMMAREQUEST"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETGAMMARESPONSE")
  (make-package "CAMERA_CONTROL_MSGS::SETGAMMARESPONSE"))

(in-package "ROS")





(defclass camera_control_msgs::SetGammaRequest
  :super ros::object
  :slots (_target_gamma ))

(defmethod camera_control_msgs::SetGammaRequest
  (:init
   (&key
    ((:target_gamma __target_gamma) 0.0)
    )
   (send-super :init)
   (setq _target_gamma (float __target_gamma))
   self)
  (:target_gamma
   (&optional __target_gamma)
   (if __target_gamma (setq _target_gamma __target_gamma)) _target_gamma)
  (:serialization-length
   ()
   (+
    ;; float32 _target_gamma
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _target_gamma
       (sys::poke _target_gamma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _target_gamma
     (setq _target_gamma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass camera_control_msgs::SetGammaResponse
  :super ros::object
  :slots (_reached_gamma _success ))

(defmethod camera_control_msgs::SetGammaResponse
  (:init
   (&key
    ((:reached_gamma __reached_gamma) 0.0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _reached_gamma (float __reached_gamma))
   (setq _success __success)
   self)
  (:reached_gamma
   (&optional __reached_gamma)
   (if __reached_gamma (setq _reached_gamma __reached_gamma)) _reached_gamma)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float32 _reached_gamma
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _reached_gamma
       (sys::poke _reached_gamma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _reached_gamma
     (setq _reached_gamma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_control_msgs::SetGamma
  :super ros::object
  :slots ())

(setf (get camera_control_msgs::SetGamma :md5sum-) "d071cb93e1f4c5737ce40debcd7942a5")
(setf (get camera_control_msgs::SetGamma :datatype-) "camera_control_msgs/SetGamma")
(setf (get camera_control_msgs::SetGamma :request) camera_control_msgs::SetGammaRequest)
(setf (get camera_control_msgs::SetGamma :response) camera_control_msgs::SetGammaResponse)

(defmethod camera_control_msgs::SetGammaRequest
  (:response () (instance camera_control_msgs::SetGammaResponse :init)))

(setf (get camera_control_msgs::SetGammaRequest :md5sum-) "d071cb93e1f4c5737ce40debcd7942a5")
(setf (get camera_control_msgs::SetGammaRequest :datatype-) "camera_control_msgs/SetGammaRequest")
(setf (get camera_control_msgs::SetGammaRequest :definition-)
      "# The target gamma correction of pixel intensity.
# Adjusts the brightness of the pixel values output by the camera's sensor
# to account for a non-linearity in the human perception of brightness or
# of the display system (such as CRT).
float32 target_gamma
---
# Exact match can not always be reached
float32 reached_gamma
bool success


")

(setf (get camera_control_msgs::SetGammaResponse :md5sum-) "d071cb93e1f4c5737ce40debcd7942a5")
(setf (get camera_control_msgs::SetGammaResponse :datatype-) "camera_control_msgs/SetGammaResponse")
(setf (get camera_control_msgs::SetGammaResponse :definition-)
      "# The target gamma correction of pixel intensity.
# Adjusts the brightness of the pixel values output by the camera's sensor
# to account for a non-linearity in the human perception of brightness or
# of the display system (such as CRT).
float32 target_gamma
---
# Exact match can not always be reached
float32 reached_gamma
bool success


")



(provide :camera_control_msgs/SetGamma "d071cb93e1f4c5737ce40debcd7942a5")


