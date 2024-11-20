;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::GrabImagesResult)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'GrabImagesResult (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::GRABIMAGESRESULT")
  (make-package "CAMERA_CONTROL_MSGS::GRABIMAGESRESULT"))

(in-package "ROS")
;;//! \htmlinclude GrabImagesResult.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass camera_control_msgs::GrabImagesResult
  :super ros::object
  :slots (_images _cam_info _reached_exposure_times _reached_brightness_values _reached_gain_values _reached_gamma_values _success ))

(defmethod camera_control_msgs::GrabImagesResult
  (:init
   (&key
    ((:images __images) ())
    ((:cam_info __cam_info) (instance sensor_msgs::CameraInfo :init))
    ((:reached_exposure_times __reached_exposure_times) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:reached_brightness_values __reached_brightness_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:reached_gain_values __reached_gain_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:reached_gamma_values __reached_gamma_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _images __images)
   (setq _cam_info __cam_info)
   (setq _reached_exposure_times __reached_exposure_times)
   (setq _reached_brightness_values __reached_brightness_values)
   (setq _reached_gain_values __reached_gain_values)
   (setq _reached_gamma_values __reached_gamma_values)
   (setq _success __success)
   self)
  (:images
   (&rest __images)
   (if (keywordp (car __images))
       (send* _images __images)
     (progn
       (if __images (setq _images (car __images)))
       _images)))
  (:cam_info
   (&rest __cam_info)
   (if (keywordp (car __cam_info))
       (send* _cam_info __cam_info)
     (progn
       (if __cam_info (setq _cam_info (car __cam_info)))
       _cam_info)))
  (:reached_exposure_times
   (&optional __reached_exposure_times)
   (if __reached_exposure_times (setq _reached_exposure_times __reached_exposure_times)) _reached_exposure_times)
  (:reached_brightness_values
   (&optional __reached_brightness_values)
   (if __reached_brightness_values (setq _reached_brightness_values __reached_brightness_values)) _reached_brightness_values)
  (:reached_gain_values
   (&optional __reached_gain_values)
   (if __reached_gain_values (setq _reached_gain_values __reached_gain_values)) _reached_gain_values)
  (:reached_gamma_values
   (&optional __reached_gamma_values)
   (if __reached_gamma_values (setq _reached_gamma_values __reached_gamma_values)) _reached_gamma_values)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image[] _images
    (apply #'+ (send-all _images :serialization-length)) 4
    ;; sensor_msgs/CameraInfo _cam_info
    (send _cam_info :serialization-length)
    ;; float32[] _reached_exposure_times
    (* 4    (length _reached_exposure_times)) 4
    ;; float32[] _reached_brightness_values
    (* 4    (length _reached_brightness_values)) 4
    ;; float32[] _reached_gain_values
    (* 4    (length _reached_gain_values)) 4
    ;; float32[] _reached_gamma_values
    (* 4    (length _reached_gamma_values)) 4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image[] _images
     (write-long (length _images) s)
     (dolist (elem _images)
       (send elem :serialize s)
       )
     ;; sensor_msgs/CameraInfo _cam_info
       (send _cam_info :serialize s)
     ;; float32[] _reached_exposure_times
     (write-long (length _reached_exposure_times) s)
     (dotimes (i (length _reached_exposure_times))
       (sys::poke (elt _reached_exposure_times i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _reached_brightness_values
     (write-long (length _reached_brightness_values) s)
     (dotimes (i (length _reached_brightness_values))
       (sys::poke (elt _reached_brightness_values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _reached_gain_values
     (write-long (length _reached_gain_values) s)
     (dotimes (i (length _reached_gain_values))
       (sys::poke (elt _reached_gain_values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _reached_gamma_values
     (write-long (length _reached_gamma_values) s)
     (dotimes (i (length _reached_gamma_values))
       (sys::poke (elt _reached_gamma_values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image[] _images
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _images (let (r) (dotimes (i n) (push (instance sensor_msgs::Image :init) r)) r))
     (dolist (elem- _images)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; sensor_msgs/CameraInfo _cam_info
     (send _cam_info :deserialize buf ptr-) (incf ptr- (send _cam_info :serialization-length))
   ;; float32[] _reached_exposure_times
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _reached_exposure_times (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _reached_exposure_times i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _reached_brightness_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _reached_brightness_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _reached_brightness_values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _reached_gain_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _reached_gain_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _reached_gain_values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _reached_gamma_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _reached_gamma_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _reached_gamma_values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get camera_control_msgs::GrabImagesResult :md5sum-) "f2d7966fcbce7df3d70e766fac52078d")
(setf (get camera_control_msgs::GrabImagesResult :datatype-) "camera_control_msgs/GrabImagesResult")
(setf (get camera_control_msgs::GrabImagesResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
##########################################
################# RESULT #################
##########################################

# The resulting images with the inquired image intensity settings.
# The size of the vector equals the size of the exposure_times or the
# brightness_values-vector
sensor_msgs/Image[] images

# The CameraInfo obejct describing the camera properties for the above image
# sequence. Static in many cases, but can also support variable binning setting
sensor_msgs/CameraInfo cam_info

# The reached values of the images e.g. the values that were set to the camera
# before the grab
float32[] reached_exposure_times

float32[] reached_brightness_values

float32[] reached_gain_values

float32[] reached_gamma_values

# Flag which indicates the success of the grabbing action
# In case of failure, the images-vector contains only the images, that could be
# grabbed before the failure occurred.
bool success

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
MSG: sensor_msgs/CameraInfo
# This message defines meta information for a camera. It should be in a
# camera namespace on topic \"camera_info\" and accompanied by up to five
# image topics named:
#
#   image_raw - raw data from the camera driver, possibly Bayer encoded
#   image            - monochrome, distorted
#   image_color      - color, distorted
#   image_rect       - monochrome, rectified
#   image_rect_color - color, rectified
#
# The image_pipeline contains packages (image_proc, stereo_image_proc)
# for producing the four processed image topics from image_raw and
# camera_info. The meaning of the camera parameters are described in
# detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
#
# The image_geometry package provides a user-friendly interface to
# common operations using this meta information. If you want to, e.g.,
# project a 3d point into image coordinates, we strongly recommend
# using image_geometry.
#
# If the camera is uncalibrated, the matrices D, K, R, P should be left
# zeroed out. In particular, clients may assume that K[0] == 0.0
# indicates an uncalibrated camera.

#######################################################################
#                     Image acquisition info                          #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
Header header    # Header timestamp should be acquisition time of image
                 # Header frame_id should be optical frame of camera
                 # origin of frame should be optical center of camera
                 # +x should point to the right in the image
                 # +y should point down in the image
                 # +z should point into the plane of the image


#######################################################################
#                      Calibration Parameters                         #
#######################################################################
# These are fixed during camera calibration. Their values will be the #
# same in all messages until the camera is recalibrated. Note that    #
# self-calibrating systems may \"recalibrate\" frequently.              #
#                                                                     #
# The internal parameters can be used to warp a raw (distorted) image #
# to:                                                                 #
#   1. An undistorted image (requires D and K)                        #
#   2. A rectified image (requires D, K, R)                           #
# The projection matrix P projects 3D points into the rectified image.#
#######################################################################

# The image dimensions with which the camera was calibrated. Normally
# this will be the full camera resolution in pixels.
uint32 height
uint32 width

# The distortion model used. Supported models are listed in
# sensor_msgs/distortion_models.h. For most cameras, \"plumb_bob\" - a
# simple model of radial and tangential distortion - is sufficient.
string distortion_model

# The distortion parameters, size depending on the distortion model.
# For \"plumb_bob\", the 5 parameters are: (k1, k2, t1, t2, k3).
float64[] D

# Intrinsic camera matrix for the raw (distorted) images.
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
# Projects 3D points in the camera coordinate frame to 2D pixel
# coordinates using the focal lengths (fx, fy) and principal point
# (cx, cy).
float64[9]  K # 3x3 row-major matrix

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
float64[9]  R # 3x3 row-major matrix

# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
# By convention, this matrix specifies the intrinsic (camera) matrix
#  of the processed (rectified) image. That is, the left 3x3 portion
#  is the normal camera intrinsic matrix for the rectified image.
# It projects 3D points in the camera coordinate frame to 2D pixel
#  coordinates using the focal lengths (fx', fy') and principal point
#  (cx', cy') - these may differ from the values in K.
# For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
#  also have R = the identity and P[1:3,1:3] = K.
# For a stereo pair, the fourth column [Tx Ty 0]' is related to the
#  position of the optical center of the second camera in the first
#  camera's frame. We assume Tz = 0 so both cameras are in the same
#  stereo image plane. The first camera always has Tx = Ty = 0. For
#  the right (second) camera of a horizontal stereo pair, Ty = 0 and
#  Tx = -fx' * B, where B is the baseline between the cameras.
# Given a 3D point [X Y Z]', the projection (x, y) of the point onto
#  the rectified image is given by:
#  [u v w]' = P * [X Y Z 1]'
#         x = u / w
#         y = v / w
#  This holds for both images of a stereo pair.
float64[12] P # 3x4 row-major matrix


#######################################################################
#                      Operational Parameters                         #
#######################################################################
# These define the image region actually captured by the camera       #
# driver. Although they affect the geometry of the output image, they #
# may be changed freely without recalibrating the camera.             #
#######################################################################

# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger \"super-pixels.\" It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
uint32 binning_x
uint32 binning_y

# Region of interest (subwindow of full camera resolution), given in
#  full resolution (unbinned) image coordinates. A particular ROI
#  always denotes the same window of pixels on the camera sensor,
#  regardless of binning settings.
# The default setting of roi (all values 0) is considered the same as
#  full resolution (roi.width = width, roi.height = height).
RegionOfInterest roi

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



(provide :camera_control_msgs/GrabImagesResult "f2d7966fcbce7df3d70e766fac52078d")


