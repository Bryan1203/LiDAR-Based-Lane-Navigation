;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::SetROI)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'SetROI (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::SETROI")
  (make-package "CAMERA_CONTROL_MSGS::SETROI"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETROIREQUEST")
  (make-package "CAMERA_CONTROL_MSGS::SETROIREQUEST"))
(unless (find-package "CAMERA_CONTROL_MSGS::SETROIRESPONSE")
  (make-package "CAMERA_CONTROL_MSGS::SETROIRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass camera_control_msgs::SetROIRequest
  :super ros::object
  :slots (_target_roi ))

(defmethod camera_control_msgs::SetROIRequest
  (:init
   (&key
    ((:target_roi __target_roi) (instance sensor_msgs::RegionOfInterest :init))
    )
   (send-super :init)
   (setq _target_roi __target_roi)
   self)
  (:target_roi
   (&rest __target_roi)
   (if (keywordp (car __target_roi))
       (send* _target_roi __target_roi)
     (progn
       (if __target_roi (setq _target_roi (car __target_roi)))
       _target_roi)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/RegionOfInterest _target_roi
    (send _target_roi :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/RegionOfInterest _target_roi
       (send _target_roi :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/RegionOfInterest _target_roi
     (send _target_roi :deserialize buf ptr-) (incf ptr- (send _target_roi :serialization-length))
   ;;
   self)
  )

(defclass camera_control_msgs::SetROIResponse
  :super ros::object
  :slots (_reached_roi _success ))

(defmethod camera_control_msgs::SetROIResponse
  (:init
   (&key
    ((:reached_roi __reached_roi) (instance sensor_msgs::RegionOfInterest :init))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _reached_roi __reached_roi)
   (setq _success __success)
   self)
  (:reached_roi
   (&rest __reached_roi)
   (if (keywordp (car __reached_roi))
       (send* _reached_roi __reached_roi)
     (progn
       (if __reached_roi (setq _reached_roi (car __reached_roi)))
       _reached_roi)))
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/RegionOfInterest _reached_roi
    (send _reached_roi :serialization-length)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/RegionOfInterest _reached_roi
       (send _reached_roi :serialize s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/RegionOfInterest _reached_roi
     (send _reached_roi :deserialize buf ptr-) (incf ptr- (send _reached_roi :serialization-length))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_control_msgs::SetROI
  :super ros::object
  :slots ())

(setf (get camera_control_msgs::SetROI :md5sum-) "58d19177e6121352c4bf9021da422622")
(setf (get camera_control_msgs::SetROI :datatype-) "camera_control_msgs/SetROI")
(setf (get camera_control_msgs::SetROI :request) camera_control_msgs::SetROIRequest)
(setf (get camera_control_msgs::SetROI :response) camera_control_msgs::SetROIResponse)

(defmethod camera_control_msgs::SetROIRequest
  (:response () (instance camera_control_msgs::SetROIResponse :init)))

(setf (get camera_control_msgs::SetROIRequest :md5sum-) "58d19177e6121352c4bf9021da422622")
(setf (get camera_control_msgs::SetROIRequest :datatype-) "camera_control_msgs/SetROIRequest")
(setf (get camera_control_msgs::SetROIRequest :definition-)
      "# Select a region of interest to get a cropped image.
# The region is defined by four parameters
# roi.width: with of the region
# roi.height: height of the region
# roi.x_offset at which pixel a long the x axis (horizontal) does the
# cropped region start
# roi.y_offset at which pixel a long the y axis (vertical) does the
# cropped region start
# The cropped image will then be Image[y_offset:y_offset+vertical, x_offset:x_offset+horizontal]
# Notice that x_offset cannot be larger than img.width - roi.width
# The same for y_offset, not larger than img.height - roi.height
sensor_msgs/RegionOfInterest target_roi


================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the \"raw\"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
---
# Exact match can not always reached
sensor_msgs/RegionOfInterest reached_roi
bool success


================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the \"raw\"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
")

(setf (get camera_control_msgs::SetROIResponse :md5sum-) "58d19177e6121352c4bf9021da422622")
(setf (get camera_control_msgs::SetROIResponse :datatype-) "camera_control_msgs/SetROIResponse")
(setf (get camera_control_msgs::SetROIResponse :definition-)
      "# Select a region of interest to get a cropped image.
# The region is defined by four parameters
# roi.width: with of the region
# roi.height: height of the region
# roi.x_offset at which pixel a long the x axis (horizontal) does the
# cropped region start
# roi.y_offset at which pixel a long the y axis (vertical) does the
# cropped region start
# The cropped image will then be Image[y_offset:y_offset+vertical, x_offset:x_offset+horizontal]
# Notice that x_offset cannot be larger than img.width - roi.width
# The same for y_offset, not larger than img.height - roi.height
sensor_msgs/RegionOfInterest target_roi


================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the \"raw\"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
---
# Exact match can not always reached
sensor_msgs/RegionOfInterest reached_roi
bool success


================================================================================
MSG: sensor_msgs/RegionOfInterest
# This message is used to specify a region of interest within an image.
#
# When used to specify the ROI setting of the camera when the image was
# taken, the height and width fields should either match the height and
# width fields for the associated image; or height = width = 0
# indicates that the full resolution image was captured.

uint32 x_offset  # Leftmost pixel of the ROI
                 # (0 if the ROI includes the left edge of the image)
uint32 y_offset  # Topmost pixel of the ROI
                 # (0 if the ROI includes the top edge of the image)
uint32 height    # Height of ROI
uint32 width     # Width of ROI

# True if a distinct rectified ROI should be calculated from the \"raw\"
# ROI in this message. Typically this should be False if the full image
# is captured (ROI not used), and True if a subwindow is captured (ROI
# used).
bool do_rectify
")



(provide :camera_control_msgs/SetROI "58d19177e6121352c4bf9021da422622")


