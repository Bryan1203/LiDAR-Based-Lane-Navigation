;; Auto-generated. Do not edit!


(when (boundp 'depthai_ros_msgs::NormalizedImageCrop)
  (if (not (find-package "DEPTHAI_ROS_MSGS"))
    (make-package "DEPTHAI_ROS_MSGS"))
  (shadow 'NormalizedImageCrop (find-package "DEPTHAI_ROS_MSGS")))
(unless (find-package "DEPTHAI_ROS_MSGS::NORMALIZEDIMAGECROP")
  (make-package "DEPTHAI_ROS_MSGS::NORMALIZEDIMAGECROP"))
(unless (find-package "DEPTHAI_ROS_MSGS::NORMALIZEDIMAGECROPREQUEST")
  (make-package "DEPTHAI_ROS_MSGS::NORMALIZEDIMAGECROPREQUEST"))
(unless (find-package "DEPTHAI_ROS_MSGS::NORMALIZEDIMAGECROPRESPONSE")
  (make-package "DEPTHAI_ROS_MSGS::NORMALIZEDIMAGECROPRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass depthai_ros_msgs::NormalizedImageCropRequest
  :super ros::object
  :slots (_top_left _bottom_right ))

(defmethod depthai_ros_msgs::NormalizedImageCropRequest
  (:init
   (&key
    ((:top_left __top_left) (instance geometry_msgs::Pose2D :init))
    ((:bottom_right __bottom_right) (instance geometry_msgs::Pose2D :init))
    )
   (send-super :init)
   (setq _top_left __top_left)
   (setq _bottom_right __bottom_right)
   self)
  (:top_left
   (&rest __top_left)
   (if (keywordp (car __top_left))
       (send* _top_left __top_left)
     (progn
       (if __top_left (setq _top_left (car __top_left)))
       _top_left)))
  (:bottom_right
   (&rest __bottom_right)
   (if (keywordp (car __bottom_right))
       (send* _bottom_right __bottom_right)
     (progn
       (if __bottom_right (setq _bottom_right (car __bottom_right)))
       _bottom_right)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose2D _top_left
    (send _top_left :serialization-length)
    ;; geometry_msgs/Pose2D _bottom_right
    (send _bottom_right :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose2D _top_left
       (send _top_left :serialize s)
     ;; geometry_msgs/Pose2D _bottom_right
       (send _bottom_right :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose2D _top_left
     (send _top_left :deserialize buf ptr-) (incf ptr- (send _top_left :serialization-length))
   ;; geometry_msgs/Pose2D _bottom_right
     (send _bottom_right :deserialize buf ptr-) (incf ptr- (send _bottom_right :serialization-length))
   ;;
   self)
  )

(defclass depthai_ros_msgs::NormalizedImageCropResponse
  :super ros::object
  :slots (_status ))

(defmethod depthai_ros_msgs::NormalizedImageCropResponse
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; int64 _status
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _status
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _status (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _status) (= (length (_status . bv)) 2)) ;; bignum
              (write-long (ash (elt (_status . bv) 0) 0) s)
              (write-long (ash (elt (_status . bv) 1) -1) s))
             ((and (class _status) (= (length (_status . bv)) 1)) ;; big1
              (write-long (elt (_status . bv) 0) s)
              (write-long (if (>= _status 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _status s)(write-long (if (>= _status 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _status
#+(or :alpha :irix6 :x86_64)
      (setf _status (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _status (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass depthai_ros_msgs::NormalizedImageCrop
  :super ros::object
  :slots ())

(setf (get depthai_ros_msgs::NormalizedImageCrop :md5sum-) "35d4b74aa140d5d6ce7c27c43314e948")
(setf (get depthai_ros_msgs::NormalizedImageCrop :datatype-) "depthai_ros_msgs/NormalizedImageCrop")
(setf (get depthai_ros_msgs::NormalizedImageCrop :request) depthai_ros_msgs::NormalizedImageCropRequest)
(setf (get depthai_ros_msgs::NormalizedImageCrop :response) depthai_ros_msgs::NormalizedImageCropResponse)

(defmethod depthai_ros_msgs::NormalizedImageCropRequest
  (:response () (instance depthai_ros_msgs::NormalizedImageCropResponse :init)))

(setf (get depthai_ros_msgs::NormalizedImageCropRequest :md5sum-) "35d4b74aa140d5d6ce7c27c43314e948")
(setf (get depthai_ros_msgs::NormalizedImageCropRequest :datatype-) "depthai_ros_msgs/NormalizedImageCropRequest")
(setf (get depthai_ros_msgs::NormalizedImageCropRequest :definition-)
      "geometry_msgs/Pose2D top_left
geometry_msgs/Pose2D bottom_right

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
---
int64 status
")

(setf (get depthai_ros_msgs::NormalizedImageCropResponse :md5sum-) "35d4b74aa140d5d6ce7c27c43314e948")
(setf (get depthai_ros_msgs::NormalizedImageCropResponse :datatype-) "depthai_ros_msgs/NormalizedImageCropResponse")
(setf (get depthai_ros_msgs::NormalizedImageCropResponse :definition-)
      "geometry_msgs/Pose2D top_left
geometry_msgs/Pose2D bottom_right

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
---
int64 status
")



(provide :depthai_ros_msgs/NormalizedImageCrop "35d4b74aa140d5d6ce7c27c43314e948")


