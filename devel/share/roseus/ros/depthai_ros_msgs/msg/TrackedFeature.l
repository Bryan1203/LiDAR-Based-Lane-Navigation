;; Auto-generated. Do not edit!


(when (boundp 'depthai_ros_msgs::TrackedFeature)
  (if (not (find-package "DEPTHAI_ROS_MSGS"))
    (make-package "DEPTHAI_ROS_MSGS"))
  (shadow 'TrackedFeature (find-package "DEPTHAI_ROS_MSGS")))
(unless (find-package "DEPTHAI_ROS_MSGS::TRACKEDFEATURE")
  (make-package "DEPTHAI_ROS_MSGS::TRACKEDFEATURE"))

(in-package "ROS")
;;//! \htmlinclude TrackedFeature.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass depthai_ros_msgs::TrackedFeature
  :super ros::object
  :slots (_header _position _id _age _harris_score _tracking_error ))

(defmethod depthai_ros_msgs::TrackedFeature
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:id __id) 0)
    ((:age __age) 0)
    ((:harris_score __harris_score) 0.0)
    ((:tracking_error __tracking_error) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _position __position)
   (setq _id (round __id))
   (setq _age (round __age))
   (setq _harris_score (float __harris_score))
   (setq _tracking_error (float __tracking_error))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:harris_score
   (&optional __harris_score)
   (if __harris_score (setq _harris_score __harris_score)) _harris_score)
  (:tracking_error
   (&optional __tracking_error)
   (if __tracking_error (setq _tracking_error __tracking_error)) _tracking_error)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; uint32 _id
    4
    ;; uint32 _age
    4
    ;; float32 _harris_score
    4
    ;; float32 _tracking_error
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; uint32 _id
       (write-long _id s)
     ;; uint32 _age
       (write-long _age s)
     ;; float32 _harris_score
       (sys::poke _harris_score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tracking_error
       (sys::poke _tracking_error (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _age
     (setq _age (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _harris_score
     (setq _harris_score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tracking_error
     (setq _tracking_error (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get depthai_ros_msgs::TrackedFeature :md5sum-) "7b24a14f9b5d900250ec5c3f7bf1ec5e")
(setf (get depthai_ros_msgs::TrackedFeature :datatype-) "depthai_ros_msgs/TrackedFeature")
(setf (get depthai_ros_msgs::TrackedFeature :definition-)
      "std_msgs/Header header

geometry_msgs/Point position
uint32 id

uint32 age

float32 harris_score

float32 tracking_error

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :depthai_ros_msgs/TrackedFeature "7b24a14f9b5d900250ec5c3f7bf1ec5e")


