;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::SetBinning)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'SetBinning (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::SETBINNING")
  (make-package "CAMERA_CONTROL_MSGS::SETBINNING"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETBINNINGREQUEST")
  (make-package "CAMERA_CONTROL_MSGS::SETBINNINGREQUEST"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETBINNINGRESPONSE")
  (make-package "CAMERA_CONTROL_MSGS::SETBINNINGRESPONSE"))

(in-package "ROS")





(defclass camera_control_msgs::SetBinningRequest
  :super ros::object
  :slots (_target_binning_x _target_binning_y ))

(defmethod camera_control_msgs::SetBinningRequest
  (:init
   (&key
    ((:target_binning_x __target_binning_x) 0)
    ((:target_binning_y __target_binning_y) 0)
    )
   (send-super :init)
   (setq _target_binning_x (round __target_binning_x))
   (setq _target_binning_y (round __target_binning_y))
   self)
  (:target_binning_x
   (&optional __target_binning_x)
   (if __target_binning_x (setq _target_binning_x __target_binning_x)) _target_binning_x)
  (:target_binning_y
   (&optional __target_binning_y)
   (if __target_binning_y (setq _target_binning_y __target_binning_y)) _target_binning_y)
  (:serialization-length
   ()
   (+
    ;; uint32 _target_binning_x
    4
    ;; uint32 _target_binning_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _target_binning_x
       (write-long _target_binning_x s)
     ;; uint32 _target_binning_y
       (write-long _target_binning_y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _target_binning_x
     (setq _target_binning_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _target_binning_y
     (setq _target_binning_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass camera_control_msgs::SetBinningResponse
  :super ros::object
  :slots (_reached_binning_x _reached_binning_y _success ))

(defmethod camera_control_msgs::SetBinningResponse
  (:init
   (&key
    ((:reached_binning_x __reached_binning_x) 0)
    ((:reached_binning_y __reached_binning_y) 0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _reached_binning_x (round __reached_binning_x))
   (setq _reached_binning_y (round __reached_binning_y))
   (setq _success __success)
   self)
  (:reached_binning_x
   (&optional __reached_binning_x)
   (if __reached_binning_x (setq _reached_binning_x __reached_binning_x)) _reached_binning_x)
  (:reached_binning_y
   (&optional __reached_binning_y)
   (if __reached_binning_y (setq _reached_binning_y __reached_binning_y)) _reached_binning_y)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; uint32 _reached_binning_x
    4
    ;; uint32 _reached_binning_y
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _reached_binning_x
       (write-long _reached_binning_x s)
     ;; uint32 _reached_binning_y
       (write-long _reached_binning_y s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _reached_binning_x
     (setq _reached_binning_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _reached_binning_y
     (setq _reached_binning_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_control_msgs::SetBinning
  :super ros::object
  :slots ())

(setf (get camera_control_msgs::SetBinning :md5sum-) "fb7b82cb93bc5513ade2d601c7a7d975")
(setf (get camera_control_msgs::SetBinning :datatype-) "camera_control_msgs/SetBinning")
(setf (get camera_control_msgs::SetBinning :request) camera_control_msgs::SetBinningRequest)
(setf (get camera_control_msgs::SetBinning :response) camera_control_msgs::SetBinningResponse)

(defmethod camera_control_msgs::SetBinningRequest
  (:response () (instance camera_control_msgs::SetBinningResponse :init)))

(setf (get camera_control_msgs::SetBinningRequest :md5sum-) "fb7b82cb93bc5513ade2d601c7a7d975")
(setf (get camera_control_msgs::SetBinningRequest :datatype-) "camera_control_msgs/SetBinningRequest")
(setf (get camera_control_msgs::SetBinningRequest :definition-)
      "# Binning factor to get downsampled images. It refers here to any camera
# setting which combines rectangular neighborhoods of pixels into larger
# \"super-pixels.\" It reduces the resolution of the output image to
# (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 are considered the same
# as binning_x = binning_y = 1 (no subsampling).
# Calling this service with target binning values will change the binning entry
# in the published camera_info_msg
uint32 target_binning_x
uint32 target_binning_y

---
# Exact match can not always be reached
uint32 reached_binning_x
uint32 reached_binning_y
bool success

")

(setf (get camera_control_msgs::SetBinningResponse :md5sum-) "fb7b82cb93bc5513ade2d601c7a7d975")
(setf (get camera_control_msgs::SetBinningResponse :datatype-) "camera_control_msgs/SetBinningResponse")
(setf (get camera_control_msgs::SetBinningResponse :definition-)
      "# Binning factor to get downsampled images. It refers here to any camera
# setting which combines rectangular neighborhoods of pixels into larger
# \"super-pixels.\" It reduces the resolution of the output image to
# (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 are considered the same
# as binning_x = binning_y = 1 (no subsampling).
# Calling this service with target binning values will change the binning entry
# in the published camera_info_msg
uint32 target_binning_x
uint32 target_binning_y

---
# Exact match can not always be reached
uint32 reached_binning_x
uint32 reached_binning_y
bool success

")



(provide :camera_control_msgs/SetBinning "fb7b82cb93bc5513ade2d601c7a7d975")


