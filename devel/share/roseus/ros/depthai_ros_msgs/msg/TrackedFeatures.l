;; Auto-generated. Do not edit!


(when (boundp 'depthai_ros_msgs::TrackedFeatures)
  (if (not (find-package "DEPTHAI_ROS_MSGS"))
    (make-package "DEPTHAI_ROS_MSGS"))
  (shadow 'TrackedFeatures (find-package "DEPTHAI_ROS_MSGS")))
(unless (find-package "DEPTHAI_ROS_MSGS::TRACKEDFEATURES")
  (make-package "DEPTHAI_ROS_MSGS::TRACKEDFEATURES"))

(in-package "ROS")
;;//! \htmlinclude TrackedFeatures.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass depthai_ros_msgs::TrackedFeatures
  :super ros::object
  :slots (_header _features ))

(defmethod depthai_ros_msgs::TrackedFeatures
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:features __features) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _features __features)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:features
   (&rest __features)
   (if (keywordp (car __features))
       (send* _features __features)
     (progn
       (if __features (setq _features (car __features)))
       _features)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; depthai_ros_msgs/TrackedFeature[] _features
    (apply #'+ (send-all _features :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; depthai_ros_msgs/TrackedFeature[] _features
     (write-long (length _features) s)
     (dolist (elem _features)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; depthai_ros_msgs/TrackedFeature[] _features
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _features (let (r) (dotimes (i n) (push (instance depthai_ros_msgs::TrackedFeature :init) r)) r))
     (dolist (elem- _features)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get depthai_ros_msgs::TrackedFeatures :md5sum-) "b205be15f3779117f010ff7ec03c8749")
(setf (get depthai_ros_msgs::TrackedFeatures :datatype-) "depthai_ros_msgs/TrackedFeatures")
(setf (get depthai_ros_msgs::TrackedFeatures :definition-)
      "std_msgs/Header header
depthai_ros_msgs/TrackedFeature[] features
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: depthai_ros_msgs/TrackedFeature
std_msgs/Header header

geometry_msgs/Point position
uint32 id

uint32 age

float32 harris_score

float32 tracking_error

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :depthai_ros_msgs/TrackedFeatures "b205be15f3779117f010ff7ec03c8749")


