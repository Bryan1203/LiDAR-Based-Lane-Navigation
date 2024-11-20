; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude RFBand.msg.html

(cl:defclass <RFBand> (roslisp-msg-protocol:ros-message)
  ((frequency
    :reader frequency
    :initarg :frequency
    :type cl:integer
    :initform 0)
   (bandwidth
    :reader bandwidth
    :initarg :bandwidth
    :type cl:fixnum
    :initform 0)
   (info
    :reader info
    :initarg :info
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RFBand (<RFBand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RFBand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RFBand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<RFBand> is deprecated: use septentrio_gnss_driver-msg:RFBand instead.")))

(cl:ensure-generic-function 'frequency-val :lambda-list '(m))
(cl:defmethod frequency-val ((m <RFBand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:frequency-val is deprecated.  Use septentrio_gnss_driver-msg:frequency instead.")
  (frequency m))

(cl:ensure-generic-function 'bandwidth-val :lambda-list '(m))
(cl:defmethod bandwidth-val ((m <RFBand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:bandwidth-val is deprecated.  Use septentrio_gnss_driver-msg:bandwidth instead.")
  (bandwidth m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <RFBand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:info-val is deprecated.  Use septentrio_gnss_driver-msg:info instead.")
  (info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RFBand>) ostream)
  "Serializes a message object of type '<RFBand>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bandwidth)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bandwidth)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'info)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RFBand>) istream)
  "Deserializes a message object of type '<RFBand>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bandwidth)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bandwidth)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'info)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RFBand>)))
  "Returns string type for a message object of type '<RFBand>"
  "septentrio_gnss_driver/RFBand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RFBand)))
  "Returns string type for a message object of type 'RFBand"
  "septentrio_gnss_driver/RFBand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RFBand>)))
  "Returns md5sum for a message object of type '<RFBand>"
  "987cd35fc563b11daae475e3e9a37fd6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RFBand)))
  "Returns md5sum for a message object of type 'RFBand"
  "987cd35fc563b11daae475e3e9a37fd6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RFBand>)))
  "Returns full string definition for message of type '<RFBand>"
  (cl:format cl:nil "# RFband block~%~%uint32 frequency # Hz~%uint16 bandwidth # kHz~%uint8  info~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RFBand)))
  "Returns full string definition for message of type 'RFBand"
  (cl:format cl:nil "# RFband block~%~%uint32 frequency # Hz~%uint16 bandwidth # kHz~%uint8  info~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RFBand>))
  (cl:+ 0
     4
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RFBand>))
  "Converts a ROS message object to a list"
  (cl:list 'RFBand
    (cl:cons ':frequency (frequency msg))
    (cl:cons ':bandwidth (bandwidth msg))
    (cl:cons ':info (info msg))
))
