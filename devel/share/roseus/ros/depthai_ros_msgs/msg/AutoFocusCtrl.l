;; Auto-generated. Do not edit!


(when (boundp 'depthai_ros_msgs::AutoFocusCtrl)
  (if (not (find-package "DEPTHAI_ROS_MSGS"))
    (make-package "DEPTHAI_ROS_MSGS"))
  (shadow 'AutoFocusCtrl (find-package "DEPTHAI_ROS_MSGS")))
(unless (find-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL")
  (make-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL"))

(in-package "ROS")
;;//! \htmlinclude AutoFocusCtrl.msg.html


(intern "*AF_MODE_AUTO*" (find-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL"))
(shadow '*AF_MODE_AUTO* (find-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL"))
(defconstant depthai_ros_msgs::AutoFocusCtrl::*AF_MODE_AUTO* 0)
(intern "*AF_MODE_MACRO*" (find-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL"))
(shadow '*AF_MODE_MACRO* (find-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL"))
(defconstant depthai_ros_msgs::AutoFocusCtrl::*AF_MODE_MACRO* 1)
(intern "*AF_MODE_CONTINUOUS_VIDEO*" (find-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL"))
(shadow '*AF_MODE_CONTINUOUS_VIDEO* (find-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL"))
(defconstant depthai_ros_msgs::AutoFocusCtrl::*AF_MODE_CONTINUOUS_VIDEO* 2)
(intern "*AF_MODE_CONTINUOUS_PICTURE*" (find-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL"))
(shadow '*AF_MODE_CONTINUOUS_PICTURE* (find-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL"))
(defconstant depthai_ros_msgs::AutoFocusCtrl::*AF_MODE_CONTINUOUS_PICTURE* 3)
(intern "*AF_MODE_EDOF*" (find-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL"))
(shadow '*AF_MODE_EDOF* (find-package "DEPTHAI_ROS_MSGS::AUTOFOCUSCTRL"))
(defconstant depthai_ros_msgs::AutoFocusCtrl::*AF_MODE_EDOF* 4)

(defun depthai_ros_msgs::AutoFocusCtrl-to-symbol (const)
  (cond
        ((= const 0) 'depthai_ros_msgs::AutoFocusCtrl::*AF_MODE_AUTO*)
        ((= const 1) 'depthai_ros_msgs::AutoFocusCtrl::*AF_MODE_MACRO*)
        ((= const 2) 'depthai_ros_msgs::AutoFocusCtrl::*AF_MODE_CONTINUOUS_VIDEO*)
        ((= const 3) 'depthai_ros_msgs::AutoFocusCtrl::*AF_MODE_CONTINUOUS_PICTURE*)
        ((= const 4) 'depthai_ros_msgs::AutoFocusCtrl::*AF_MODE_EDOF*)
        (t nil)))

(defclass depthai_ros_msgs::AutoFocusCtrl
  :super ros::object
  :slots (_auto_focus_mode _trigger_auto_focus ))

(defmethod depthai_ros_msgs::AutoFocusCtrl
  (:init
   (&key
    ((:auto_focus_mode __auto_focus_mode) 0)
    ((:trigger_auto_focus __trigger_auto_focus) nil)
    )
   (send-super :init)
   (setq _auto_focus_mode (round __auto_focus_mode))
   (setq _trigger_auto_focus __trigger_auto_focus)
   self)
  (:auto_focus_mode
   (&optional __auto_focus_mode)
   (if __auto_focus_mode (setq _auto_focus_mode __auto_focus_mode)) _auto_focus_mode)
  (:trigger_auto_focus
   (&optional (__trigger_auto_focus :null))
   (if (not (eq __trigger_auto_focus :null)) (setq _trigger_auto_focus __trigger_auto_focus)) _trigger_auto_focus)
  (:serialization-length
   ()
   (+
    ;; uint8 _auto_focus_mode
    1
    ;; bool _trigger_auto_focus
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _auto_focus_mode
       (write-byte _auto_focus_mode s)
     ;; bool _trigger_auto_focus
       (if _trigger_auto_focus (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _auto_focus_mode
     (setq _auto_focus_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _trigger_auto_focus
     (setq _trigger_auto_focus (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get depthai_ros_msgs::AutoFocusCtrl :md5sum-) "57bd33867f64daa701fce3a114856f8f")
(setf (get depthai_ros_msgs::AutoFocusCtrl :datatype-) "depthai_ros_msgs/AutoFocusCtrl")
(setf (get depthai_ros_msgs::AutoFocusCtrl :definition-)
      "uint8 AF_MODE_AUTO = 0
uint8 AF_MODE_MACRO = 1
uint8 AF_MODE_CONTINUOUS_VIDEO = 2
uint8 AF_MODE_CONTINUOUS_PICTURE = 3
uint8 AF_MODE_EDOF = 4

uint8 auto_focus_mode

bool trigger_auto_focus

")



(provide :depthai_ros_msgs/AutoFocusCtrl "57bd33867f64daa701fce3a114856f8f")


