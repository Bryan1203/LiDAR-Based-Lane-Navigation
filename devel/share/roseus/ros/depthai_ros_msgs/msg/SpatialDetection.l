;; Auto-generated. Do not edit!


(when (boundp 'depthai_ros_msgs::SpatialDetection)
  (if (not (find-package "DEPTHAI_ROS_MSGS"))
    (make-package "DEPTHAI_ROS_MSGS"))
  (shadow 'SpatialDetection (find-package "DEPTHAI_ROS_MSGS")))
(unless (find-package "DEPTHAI_ROS_MSGS::SPATIALDETECTION")
  (make-package "DEPTHAI_ROS_MSGS::SPATIALDETECTION"))

(in-package "ROS")
;;//! \htmlinclude SpatialDetection.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "VISION_MSGS"))
  (ros::roseus-add-msgs "vision_msgs"))


(defclass depthai_ros_msgs::SpatialDetection
  :super ros::object
  :slots (_results _bbox _position _is_tracking _tracking_id ))

(defmethod depthai_ros_msgs::SpatialDetection
  (:init
   (&key
    ((:results __results) ())
    ((:bbox __bbox) (instance vision_msgs::BoundingBox2D :init))
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:is_tracking __is_tracking) nil)
    ((:tracking_id __tracking_id) "")
    )
   (send-super :init)
   (setq _results __results)
   (setq _bbox __bbox)
   (setq _position __position)
   (setq _is_tracking __is_tracking)
   (setq _tracking_id (string __tracking_id))
   self)
  (:results
   (&rest __results)
   (if (keywordp (car __results))
       (send* _results __results)
     (progn
       (if __results (setq _results (car __results)))
       _results)))
  (:bbox
   (&rest __bbox)
   (if (keywordp (car __bbox))
       (send* _bbox __bbox)
     (progn
       (if __bbox (setq _bbox (car __bbox)))
       _bbox)))
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:is_tracking
   (&optional (__is_tracking :null))
   (if (not (eq __is_tracking :null)) (setq _is_tracking __is_tracking)) _is_tracking)
  (:tracking_id
   (&optional __tracking_id)
   (if __tracking_id (setq _tracking_id __tracking_id)) _tracking_id)
  (:serialization-length
   ()
   (+
    ;; vision_msgs/ObjectHypothesis[] _results
    (apply #'+ (send-all _results :serialization-length)) 4
    ;; vision_msgs/BoundingBox2D _bbox
    (send _bbox :serialization-length)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; bool _is_tracking
    1
    ;; string _tracking_id
    4 (length _tracking_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; vision_msgs/ObjectHypothesis[] _results
     (write-long (length _results) s)
     (dolist (elem _results)
       (send elem :serialize s)
       )
     ;; vision_msgs/BoundingBox2D _bbox
       (send _bbox :serialize s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; bool _is_tracking
       (if _is_tracking (write-byte -1 s) (write-byte 0 s))
     ;; string _tracking_id
       (write-long (length _tracking_id) s) (princ _tracking_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; vision_msgs/ObjectHypothesis[] _results
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _results (let (r) (dotimes (i n) (push (instance vision_msgs::ObjectHypothesis :init) r)) r))
     (dolist (elem- _results)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; vision_msgs/BoundingBox2D _bbox
     (send _bbox :deserialize buf ptr-) (incf ptr- (send _bbox :serialization-length))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; bool _is_tracking
     (setq _is_tracking (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _tracking_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _tracking_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get depthai_ros_msgs::SpatialDetection :md5sum-) "503c7980b555f0fd79e92d14cb9ac446")
(setf (get depthai_ros_msgs::SpatialDetection :datatype-) "depthai_ros_msgs/SpatialDetection")
(setf (get depthai_ros_msgs::SpatialDetection :definition-)
      "
# Class probabilities
vision_msgs/ObjectHypothesis[] results

# 2D bounding box surrounding the object.
vision_msgs/BoundingBox2D bbox

# Center of the detected object in meters 
geometry_msgs/Point position

# If true, this message contains object tracking information.
bool is_tracking

# ID used for consistency across multiple detection messages. This value will
# likely differ from the id field set in each individual ObjectHypothesis.
# If you set this field, be sure to also set is_tracking to True.
string tracking_id
================================================================================
MSG: vision_msgs/ObjectHypothesis
# An object hypothesis that contains no position information.

# The unique numeric ID of object detected. To get additional information about
#   this ID, such as its human-readable name, listeners should perform a lookup
#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
int64 id

# The probability or confidence value of the detected object. By convention,
#   this value should lie in the range [0-1].
float64 score
================================================================================
MSG: vision_msgs/BoundingBox2D
# A 2D bounding box that can be rotated about its center.
# All dimensions are in pixels, but represented using floating-point
#   values to allow sub-pixel precision. If an exact pixel crop is required
#   for a rotated bounding box, it can be calculated using Bresenham's line
#   algorithm.

# The 2D position (in pixels) and orientation of the bounding box center.
geometry_msgs/Pose2D center

# The size (in pixels) of the bounding box surrounding the object relative
#   to the pose of its center.
float64 size_x
float64 size_y

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



(provide :depthai_ros_msgs/SpatialDetection "503c7980b555f0fd79e92d14cb9ac446")


