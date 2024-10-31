; Auto-generated. Do not edit!


(cl:in-package camera_control_msgs-srv)


;//! \htmlinclude SetROI-request.msg.html

(cl:defclass <SetROI-request> (roslisp-msg-protocol:ros-message)
  ((target_roi
    :reader target_roi
    :initarg :target_roi
    :type sensor_msgs-msg:RegionOfInterest
    :initform (cl:make-instance 'sensor_msgs-msg:RegionOfInterest)))
)

(cl:defclass SetROI-request (<SetROI-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetROI-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetROI-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetROI-request> is deprecated: use camera_control_msgs-srv:SetROI-request instead.")))

(cl:ensure-generic-function 'target_roi-val :lambda-list '(m))
(cl:defmethod target_roi-val ((m <SetROI-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:target_roi-val is deprecated.  Use camera_control_msgs-srv:target_roi instead.")
  (target_roi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetROI-request>) ostream)
  "Serializes a message object of type '<SetROI-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_roi) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetROI-request>) istream)
  "Deserializes a message object of type '<SetROI-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_roi) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetROI-request>)))
  "Returns string type for a service object of type '<SetROI-request>"
  "camera_control_msgs/SetROIRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetROI-request)))
  "Returns string type for a service object of type 'SetROI-request"
  "camera_control_msgs/SetROIRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetROI-request>)))
  "Returns md5sum for a message object of type '<SetROI-request>"
  "58d19177e6121352c4bf9021da422622")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetROI-request)))
  "Returns md5sum for a message object of type 'SetROI-request"
  "58d19177e6121352c4bf9021da422622")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetROI-request>)))
  "Returns full string definition for message of type '<SetROI-request>"
  (cl:format cl:nil "# Select a region of interest to get a cropped image.~%# The region is defined by four parameters~%# roi.width: with of the region~%# roi.height: height of the region~%# roi.x_offset at which pixel a long the x axis (horizontal) does the~%# cropped region start~%# roi.y_offset at which pixel a long the y axis (vertical) does the~%# cropped region start~%# The cropped image will then be Image[y_offset:y_offset+vertical, x_offset:x_offset+horizontal]~%# Notice that x_offset cannot be larger than img.width - roi.width~%# The same for y_offset, not larger than img.height - roi.height~%sensor_msgs/RegionOfInterest target_roi~%~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetROI-request)))
  "Returns full string definition for message of type 'SetROI-request"
  (cl:format cl:nil "# Select a region of interest to get a cropped image.~%# The region is defined by four parameters~%# roi.width: with of the region~%# roi.height: height of the region~%# roi.x_offset at which pixel a long the x axis (horizontal) does the~%# cropped region start~%# roi.y_offset at which pixel a long the y axis (vertical) does the~%# cropped region start~%# The cropped image will then be Image[y_offset:y_offset+vertical, x_offset:x_offset+horizontal]~%# Notice that x_offset cannot be larger than img.width - roi.width~%# The same for y_offset, not larger than img.height - roi.height~%sensor_msgs/RegionOfInterest target_roi~%~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetROI-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_roi))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetROI-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetROI-request
    (cl:cons ':target_roi (target_roi msg))
))
;//! \htmlinclude SetROI-response.msg.html

(cl:defclass <SetROI-response> (roslisp-msg-protocol:ros-message)
  ((reached_roi
    :reader reached_roi
    :initarg :reached_roi
    :type sensor_msgs-msg:RegionOfInterest
    :initform (cl:make-instance 'sensor_msgs-msg:RegionOfInterest))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetROI-response (<SetROI-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetROI-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetROI-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera_control_msgs-srv:<SetROI-response> is deprecated: use camera_control_msgs-srv:SetROI-response instead.")))

(cl:ensure-generic-function 'reached_roi-val :lambda-list '(m))
(cl:defmethod reached_roi-val ((m <SetROI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:reached_roi-val is deprecated.  Use camera_control_msgs-srv:reached_roi instead.")
  (reached_roi m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetROI-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera_control_msgs-srv:success-val is deprecated.  Use camera_control_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetROI-response>) ostream)
  "Serializes a message object of type '<SetROI-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'reached_roi) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetROI-response>) istream)
  "Deserializes a message object of type '<SetROI-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'reached_roi) istream)
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetROI-response>)))
  "Returns string type for a service object of type '<SetROI-response>"
  "camera_control_msgs/SetROIResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetROI-response)))
  "Returns string type for a service object of type 'SetROI-response"
  "camera_control_msgs/SetROIResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetROI-response>)))
  "Returns md5sum for a message object of type '<SetROI-response>"
  "58d19177e6121352c4bf9021da422622")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetROI-response)))
  "Returns md5sum for a message object of type 'SetROI-response"
  "58d19177e6121352c4bf9021da422622")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetROI-response>)))
  "Returns full string definition for message of type '<SetROI-response>"
  (cl:format cl:nil "# Exact match can not always reached~%sensor_msgs/RegionOfInterest reached_roi~%bool success~%~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetROI-response)))
  "Returns full string definition for message of type 'SetROI-response"
  (cl:format cl:nil "# Exact match can not always reached~%sensor_msgs/RegionOfInterest reached_roi~%bool success~%~%~%================================================================================~%MSG: sensor_msgs/RegionOfInterest~%# This message is used to specify a region of interest within an image.~%#~%# When used to specify the ROI setting of the camera when the image was~%# taken, the height and width fields should either match the height and~%# width fields for the associated image; or height = width = 0~%# indicates that the full resolution image was captured.~%~%uint32 x_offset  # Leftmost pixel of the ROI~%                 # (0 if the ROI includes the left edge of the image)~%uint32 y_offset  # Topmost pixel of the ROI~%                 # (0 if the ROI includes the top edge of the image)~%uint32 height    # Height of ROI~%uint32 width     # Width of ROI~%~%# True if a distinct rectified ROI should be calculated from the \"raw\"~%# ROI in this message. Typically this should be False if the full image~%# is captured (ROI not used), and True if a subwindow is captured (ROI~%# used).~%bool do_rectify~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetROI-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'reached_roi))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetROI-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetROI-response
    (cl:cons ':reached_roi (reached_roi msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetROI)))
  'SetROI-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetROI)))
  'SetROI-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetROI)))
  "Returns string type for a service object of type '<SetROI>"
  "camera_control_msgs/SetROI")