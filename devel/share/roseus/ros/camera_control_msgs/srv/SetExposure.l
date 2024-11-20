;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::SetExposure)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'SetExposure (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::SETEXPOSURE")
  (make-package "CAMERA_CONTROL_MSGS::SETEXPOSURE"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETEXPOSUREREQUEST")
  (make-package "CAMERA_CONTROL_MSGS::SETEXPOSUREREQUEST"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETEXPOSURERESPONSE")
  (make-package "CAMERA_CONTROL_MSGS::SETEXPOSURERESPONSE"))

(in-package "ROS")





(defclass camera_control_msgs::SetExposureRequest
  :super ros::object
  :slots (_target_exposure ))

(defmethod camera_control_msgs::SetExposureRequest
  (:init
   (&key
    ((:target_exposure __target_exposure) 0.0)
    )
   (send-super :init)
   (setq _target_exposure (float __target_exposure))
   self)
  (:target_exposure
   (&optional __target_exposure)
   (if __target_exposure (setq _target_exposure __target_exposure)) _target_exposure)
  (:serialization-length
   ()
   (+
    ;; float32 _target_exposure
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _target_exposure
       (sys::poke _target_exposure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _target_exposure
     (setq _target_exposure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass camera_control_msgs::SetExposureResponse
  :super ros::object
  :slots (_reached_exposure _success ))

(defmethod camera_control_msgs::SetExposureResponse
  (:init
   (&key
    ((:reached_exposure __reached_exposure) 0.0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _reached_exposure (float __reached_exposure))
   (setq _success __success)
   self)
  (:reached_exposure
   (&optional __reached_exposure)
   (if __reached_exposure (setq _reached_exposure __reached_exposure)) _reached_exposure)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float32 _reached_exposure
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _reached_exposure
       (sys::poke _reached_exposure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _reached_exposure
     (setq _reached_exposure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_control_msgs::SetExposure
  :super ros::object
  :slots ())

(setf (get camera_control_msgs::SetExposure :md5sum-) "07036591479e56cbaa8b13d231a2c756")
(setf (get camera_control_msgs::SetExposure :datatype-) "camera_control_msgs/SetExposure")
(setf (get camera_control_msgs::SetExposure :request) camera_control_msgs::SetExposureRequest)
(setf (get camera_control_msgs::SetExposure :response) camera_control_msgs::SetExposureResponse)

(defmethod camera_control_msgs::SetExposureRequest
  (:response () (instance camera_control_msgs::SetExposureResponse :init)))

(setf (get camera_control_msgs::SetExposureRequest :md5sum-) "07036591479e56cbaa8b13d231a2c756")
(setf (get camera_control_msgs::SetExposureRequest :datatype-) "camera_control_msgs/SetExposureRequest")
(setf (get camera_control_msgs::SetExposureRequest :definition-)
      "# The target exposure time measured in microseconds.
# If the limits were exceeded, the desired exposure time will be truncated.
float32 target_exposure
---
# Exact match can not always be reached
float32 reached_exposure
bool success

")

(setf (get camera_control_msgs::SetExposureResponse :md5sum-) "07036591479e56cbaa8b13d231a2c756")
(setf (get camera_control_msgs::SetExposureResponse :datatype-) "camera_control_msgs/SetExposureResponse")
(setf (get camera_control_msgs::SetExposureResponse :definition-)
      "# The target exposure time measured in microseconds.
# If the limits were exceeded, the desired exposure time will be truncated.
float32 target_exposure
---
# Exact match can not always be reached
float32 reached_exposure
bool success

")



(provide :camera_control_msgs/SetExposure "07036591479e56cbaa8b13d231a2c756")


