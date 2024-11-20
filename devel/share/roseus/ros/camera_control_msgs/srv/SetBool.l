;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::SetBool)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'SetBool (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::SETBOOL")
  (make-package "CAMERA_CONTROL_MSGS::SETBOOL"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETBOOLREQUEST")
  (make-package "CAMERA_CONTROL_MSGS::SETBOOLREQUEST"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETBOOLRESPONSE")
  (make-package "CAMERA_CONTROL_MSGS::SETBOOLRESPONSE"))

(in-package "ROS")





(defclass camera_control_msgs::SetBoolRequest
  :super ros::object
  :slots (_data ))

(defmethod camera_control_msgs::SetBoolRequest
  (:init
   (&key
    ((:data __data) nil)
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional (__data :null))
   (if (not (eq __data :null)) (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; bool _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _data
       (if _data (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _data
     (setq _data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_control_msgs::SetBoolResponse
  :super ros::object
  :slots (_success _message ))

(defmethod camera_control_msgs::SetBoolResponse
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

(defclass camera_control_msgs::SetBool
  :super ros::object
  :slots ())

(setf (get camera_control_msgs::SetBool :md5sum-) "09fb03525b03e7ea1fd3992bafd87e16")
(setf (get camera_control_msgs::SetBool :datatype-) "camera_control_msgs/SetBool")
(setf (get camera_control_msgs::SetBool :request) camera_control_msgs::SetBoolRequest)
(setf (get camera_control_msgs::SetBool :response) camera_control_msgs::SetBoolResponse)

(defmethod camera_control_msgs::SetBoolRequest
  (:response () (instance camera_control_msgs::SetBoolResponse :init)))

(setf (get camera_control_msgs::SetBoolRequest :md5sum-) "09fb03525b03e7ea1fd3992bafd87e16")
(setf (get camera_control_msgs::SetBoolRequest :datatype-) "camera_control_msgs/SetBoolRequest")
(setf (get camera_control_msgs::SetBoolRequest :definition-)
      "# Borrowed from std_srvs/SetBool until they release the package

bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")

(setf (get camera_control_msgs::SetBoolResponse :md5sum-) "09fb03525b03e7ea1fd3992bafd87e16")
(setf (get camera_control_msgs::SetBoolResponse :datatype-) "camera_control_msgs/SetBoolResponse")
(setf (get camera_control_msgs::SetBoolResponse :definition-)
      "# Borrowed from std_srvs/SetBool until they release the package

bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

")



(provide :camera_control_msgs/SetBool "09fb03525b03e7ea1fd3992bafd87e16")


