;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::SetSleeping)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'SetSleeping (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::SETSLEEPING")
  (make-package "CAMERA_CONTROL_MSGS::SETSLEEPING"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETSLEEPINGREQUEST")
  (make-package "CAMERA_CONTROL_MSGS::SETSLEEPINGREQUEST"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETSLEEPINGRESPONSE")
  (make-package "CAMERA_CONTROL_MSGS::SETSLEEPINGRESPONSE"))

(in-package "ROS")





(defclass camera_control_msgs::SetSleepingRequest
  :super ros::object
  :slots (_set_sleeping ))

(defmethod camera_control_msgs::SetSleepingRequest
  (:init
   (&key
    ((:set_sleeping __set_sleeping) nil)
    )
   (send-super :init)
   (setq _set_sleeping __set_sleeping)
   self)
  (:set_sleeping
   (&optional (__set_sleeping :null))
   (if (not (eq __set_sleeping :null)) (setq _set_sleeping __set_sleeping)) _set_sleeping)
  (:serialization-length
   ()
   (+
    ;; bool _set_sleeping
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _set_sleeping
       (if _set_sleeping (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _set_sleeping
     (setq _set_sleeping (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_control_msgs::SetSleepingResponse
  :super ros::object
  :slots (_success ))

(defmethod camera_control_msgs::SetSleepingResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_control_msgs::SetSleeping
  :super ros::object
  :slots ())

(setf (get camera_control_msgs::SetSleeping :md5sum-) "f51754709312645db79ada18bc0254a3")
(setf (get camera_control_msgs::SetSleeping :datatype-) "camera_control_msgs/SetSleeping")
(setf (get camera_control_msgs::SetSleeping :request) camera_control_msgs::SetSleepingRequest)
(setf (get camera_control_msgs::SetSleeping :response) camera_control_msgs::SetSleepingResponse)

(defmethod camera_control_msgs::SetSleepingRequest
  (:response () (instance camera_control_msgs::SetSleepingResponse :init)))

(setf (get camera_control_msgs::SetSleepingRequest :md5sum-) "f51754709312645db79ada18bc0254a3")
(setf (get camera_control_msgs::SetSleepingRequest :datatype-) "camera_control_msgs/SetSleepingRequest")
(setf (get camera_control_msgs::SetSleepingRequest :definition-)
      "# If the camera runs in topic mode (continuously publishing images over the
# topics respecting the desired frame_rate) this service offers the posibillity
# to pause the image acquisition. To restart the grabbing, this service should
# be called again with set_sleeping set to false
bool set_sleeping
---
bool success

")

(setf (get camera_control_msgs::SetSleepingResponse :md5sum-) "f51754709312645db79ada18bc0254a3")
(setf (get camera_control_msgs::SetSleepingResponse :datatype-) "camera_control_msgs/SetSleepingResponse")
(setf (get camera_control_msgs::SetSleepingResponse :definition-)
      "# If the camera runs in topic mode (continuously publishing images over the
# topics respecting the desired frame_rate) this service offers the posibillity
# to pause the image acquisition. To restart the grabbing, this service should
# be called again with set_sleeping set to false
bool set_sleeping
---
bool success

")



(provide :camera_control_msgs/SetSleeping "f51754709312645db79ada18bc0254a3")


