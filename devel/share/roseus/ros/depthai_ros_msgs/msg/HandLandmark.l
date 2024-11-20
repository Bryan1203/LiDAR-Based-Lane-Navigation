;; Auto-generated. Do not edit!


(when (boundp 'depthai_ros_msgs::HandLandmark)
  (if (not (find-package "DEPTHAI_ROS_MSGS"))
    (make-package "DEPTHAI_ROS_MSGS"))
  (shadow 'HandLandmark (find-package "DEPTHAI_ROS_MSGS")))
(unless (find-package "DEPTHAI_ROS_MSGS::HANDLANDMARK")
  (make-package "DEPTHAI_ROS_MSGS::HANDLANDMARK"))

(in-package "ROS")
;;//! \htmlinclude HandLandmark.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass depthai_ros_msgs::HandLandmark
  :super ros::object
  :slots (_label _lm_score _landmark _position _is_spatial ))

(defmethod depthai_ros_msgs::HandLandmark
  (:init
   (&key
    ((:label __label) "")
    ((:lm_score __lm_score) 0.0)
    ((:landmark __landmark) ())
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:is_spatial __is_spatial) nil)
    )
   (send-super :init)
   (setq _label (string __label))
   (setq _lm_score (float __lm_score))
   (setq _landmark __landmark)
   (setq _position __position)
   (setq _is_spatial __is_spatial)
   self)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:lm_score
   (&optional __lm_score)
   (if __lm_score (setq _lm_score __lm_score)) _lm_score)
  (:landmark
   (&rest __landmark)
   (if (keywordp (car __landmark))
       (send* _landmark __landmark)
     (progn
       (if __landmark (setq _landmark (car __landmark)))
       _landmark)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:is_spatial
   (&optional (__is_spatial :null))
   (if (not (eq __is_spatial :null)) (setq _is_spatial __is_spatial)) _is_spatial)
  (:serialization-length
   ()
   (+
    ;; string _label
    4 (length _label)
    ;; float32 _lm_score
    4
    ;; geometry_msgs/Pose2D[] _landmark
    (apply #'+ (send-all _landmark :serialization-length)) 4
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; bool _is_spatial
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float32 _lm_score
       (sys::poke _lm_score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Pose2D[] _landmark
     (write-long (length _landmark) s)
     (dolist (elem _landmark)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; bool _is_spatial
       (if _is_spatial (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _lm_score
     (setq _lm_score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Pose2D[] _landmark
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _landmark (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose2D :init) r)) r))
     (dolist (elem- _landmark)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; bool _is_spatial
     (setq _is_spatial (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get depthai_ros_msgs::HandLandmark :md5sum-) "e126f09ad65617304665e31f1a5736de")
(setf (get depthai_ros_msgs::HandLandmark :datatype-) "depthai_ros_msgs/HandLandmark")
(setf (get depthai_ros_msgs::HandLandmark :definition-)
      "
# Center of the 

string label

# Landmarks score.
float32 lm_score

geometry_msgs/Pose2D[] landmark

geometry_msgs/Point position
bool is_spatial

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :depthai_ros_msgs/HandLandmark "e126f09ad65617304665e31f1a5736de")


