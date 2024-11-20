;; Auto-generated. Do not edit!


(when (boundp 'depthai_ros_msgs::TriggerNamed)
  (if (not (find-package "DEPTHAI_ROS_MSGS"))
    (make-package "DEPTHAI_ROS_MSGS"))
  (shadow 'TriggerNamed (find-package "DEPTHAI_ROS_MSGS")))
(unless (find-package "DEPTHAI_ROS_MSGS::TRIGGERNAMED")
  (make-package "DEPTHAI_ROS_MSGS::TRIGGERNAMED"))
(unless (find-package "DEPTHAI_ROS_MSGS::TRIGGERNAMEDREQUEST")
  (make-package "DEPTHAI_ROS_MSGS::TRIGGERNAMEDREQUEST"))
(unless (find-package "DEPTHAI_ROS_MSGS::TRIGGERNAMEDRESPONSE")
  (make-package "DEPTHAI_ROS_MSGS::TRIGGERNAMEDRESPONSE"))

(in-package "ROS")





(defclass depthai_ros_msgs::TriggerNamedRequest
  :super ros::object
  :slots (_name ))

(defmethod depthai_ros_msgs::TriggerNamedRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass depthai_ros_msgs::TriggerNamedResponse
  :super ros::object
  :slots (_success _message ))

(defmethod depthai_ros_msgs::TriggerNamedResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass depthai_ros_msgs::TriggerNamed
  :super ros::object
  :slots ())

(setf (get depthai_ros_msgs::TriggerNamed :md5sum-) "d82dc6474dd88dad5e1615ab1b2ca74c")
(setf (get depthai_ros_msgs::TriggerNamed :datatype-) "depthai_ros_msgs/TriggerNamed")
(setf (get depthai_ros_msgs::TriggerNamed :request) depthai_ros_msgs::TriggerNamedRequest)
(setf (get depthai_ros_msgs::TriggerNamed :response) depthai_ros_msgs::TriggerNamedResponse)

(defmethod depthai_ros_msgs::TriggerNamedRequest
  (:response () (instance depthai_ros_msgs::TriggerNamedResponse :init)))

(setf (get depthai_ros_msgs::TriggerNamedRequest :md5sum-) "d82dc6474dd88dad5e1615ab1b2ca74c")
(setf (get depthai_ros_msgs::TriggerNamedRequest :datatype-) "depthai_ros_msgs/TriggerNamedRequest")
(setf (get depthai_ros_msgs::TriggerNamedRequest :definition-)
      "string name
---
bool success
string message

")

(setf (get depthai_ros_msgs::TriggerNamedResponse :md5sum-) "d82dc6474dd88dad5e1615ab1b2ca74c")
(setf (get depthai_ros_msgs::TriggerNamedResponse :datatype-) "depthai_ros_msgs/TriggerNamedResponse")
(setf (get depthai_ros_msgs::TriggerNamedResponse :definition-)
      "string name
---
bool success
string message

")



(provide :depthai_ros_msgs/TriggerNamed "d82dc6474dd88dad5e1615ab1b2ca74c")


