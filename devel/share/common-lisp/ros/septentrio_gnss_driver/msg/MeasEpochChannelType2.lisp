; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude MeasEpochChannelType2.msg.html

(cl:defclass <MeasEpochChannelType2> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (lock_time
    :reader lock_time
    :initarg :lock_time
    :type cl:fixnum
    :initform 0)
   (cn0
    :reader cn0
    :initarg :cn0
    :type cl:fixnum
    :initform 0)
   (offsets_msb
    :reader offsets_msb
    :initarg :offsets_msb
    :type cl:fixnum
    :initform 0)
   (carrier_msb
    :reader carrier_msb
    :initarg :carrier_msb
    :type cl:fixnum
    :initform 0)
   (obs_info
    :reader obs_info
    :initarg :obs_info
    :type cl:fixnum
    :initform 0)
   (code_offset_lsb
    :reader code_offset_lsb
    :initarg :code_offset_lsb
    :type cl:fixnum
    :initform 0)
   (carrier_lsb
    :reader carrier_lsb
    :initarg :carrier_lsb
    :type cl:fixnum
    :initform 0)
   (doppler_offset_lsb
    :reader doppler_offset_lsb
    :initarg :doppler_offset_lsb
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MeasEpochChannelType2 (<MeasEpochChannelType2>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MeasEpochChannelType2>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MeasEpochChannelType2)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<MeasEpochChannelType2> is deprecated: use septentrio_gnss_driver-msg:MeasEpochChannelType2 instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <MeasEpochChannelType2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:type-val is deprecated.  Use septentrio_gnss_driver-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'lock_time-val :lambda-list '(m))
(cl:defmethod lock_time-val ((m <MeasEpochChannelType2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:lock_time-val is deprecated.  Use septentrio_gnss_driver-msg:lock_time instead.")
  (lock_time m))

(cl:ensure-generic-function 'cn0-val :lambda-list '(m))
(cl:defmethod cn0-val ((m <MeasEpochChannelType2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cn0-val is deprecated.  Use septentrio_gnss_driver-msg:cn0 instead.")
  (cn0 m))

(cl:ensure-generic-function 'offsets_msb-val :lambda-list '(m))
(cl:defmethod offsets_msb-val ((m <MeasEpochChannelType2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:offsets_msb-val is deprecated.  Use septentrio_gnss_driver-msg:offsets_msb instead.")
  (offsets_msb m))

(cl:ensure-generic-function 'carrier_msb-val :lambda-list '(m))
(cl:defmethod carrier_msb-val ((m <MeasEpochChannelType2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:carrier_msb-val is deprecated.  Use septentrio_gnss_driver-msg:carrier_msb instead.")
  (carrier_msb m))

(cl:ensure-generic-function 'obs_info-val :lambda-list '(m))
(cl:defmethod obs_info-val ((m <MeasEpochChannelType2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:obs_info-val is deprecated.  Use septentrio_gnss_driver-msg:obs_info instead.")
  (obs_info m))

(cl:ensure-generic-function 'code_offset_lsb-val :lambda-list '(m))
(cl:defmethod code_offset_lsb-val ((m <MeasEpochChannelType2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:code_offset_lsb-val is deprecated.  Use septentrio_gnss_driver-msg:code_offset_lsb instead.")
  (code_offset_lsb m))

(cl:ensure-generic-function 'carrier_lsb-val :lambda-list '(m))
(cl:defmethod carrier_lsb-val ((m <MeasEpochChannelType2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:carrier_lsb-val is deprecated.  Use septentrio_gnss_driver-msg:carrier_lsb instead.")
  (carrier_lsb m))

(cl:ensure-generic-function 'doppler_offset_lsb-val :lambda-list '(m))
(cl:defmethod doppler_offset_lsb-val ((m <MeasEpochChannelType2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:doppler_offset_lsb-val is deprecated.  Use septentrio_gnss_driver-msg:doppler_offset_lsb instead.")
  (doppler_offset_lsb m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MeasEpochChannelType2>) ostream)
  "Serializes a message object of type '<MeasEpochChannelType2>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lock_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cn0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'offsets_msb)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'carrier_msb)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obs_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'code_offset_lsb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'code_offset_lsb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'carrier_lsb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'carrier_lsb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'doppler_offset_lsb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'doppler_offset_lsb)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MeasEpochChannelType2>) istream)
  "Deserializes a message object of type '<MeasEpochChannelType2>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lock_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cn0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'offsets_msb)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'carrier_msb) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obs_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'code_offset_lsb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'code_offset_lsb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'carrier_lsb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'carrier_lsb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'doppler_offset_lsb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'doppler_offset_lsb)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MeasEpochChannelType2>)))
  "Returns string type for a message object of type '<MeasEpochChannelType2>"
  "septentrio_gnss_driver/MeasEpochChannelType2")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MeasEpochChannelType2)))
  "Returns string type for a message object of type 'MeasEpochChannelType2"
  "septentrio_gnss_driver/MeasEpochChannelType2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MeasEpochChannelType2>)))
  "Returns md5sum for a message object of type '<MeasEpochChannelType2>"
  "204efbe40d2e56cd210b50c219f3a400")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MeasEpochChannelType2)))
  "Returns md5sum for a message object of type 'MeasEpochChannelType2"
  "204efbe40d2e56cd210b50c219f3a400")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MeasEpochChannelType2>)))
  "Returns full string definition for message of type '<MeasEpochChannelType2>"
  (cl:format cl:nil "# MeasEpochChannelType2 block~%   ~%uint8  type~%uint8  lock_time          # s~%uint8  cn0                # 0.25 dB-Hz~%uint8  offsets_msb        # 65.536 m or 65.536 Hz~%int8   carrier_msb        # 65.536 cycles~%uint8  obs_info~%uint16 code_offset_lsb    # 0.001 m~%uint16 carrier_lsb        # 0.001 cycles~%uint16 doppler_offset_lsb # 0.0001 Hz~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MeasEpochChannelType2)))
  "Returns full string definition for message of type 'MeasEpochChannelType2"
  (cl:format cl:nil "# MeasEpochChannelType2 block~%   ~%uint8  type~%uint8  lock_time          # s~%uint8  cn0                # 0.25 dB-Hz~%uint8  offsets_msb        # 65.536 m or 65.536 Hz~%int8   carrier_msb        # 65.536 cycles~%uint8  obs_info~%uint16 code_offset_lsb    # 0.001 m~%uint16 carrier_lsb        # 0.001 cycles~%uint16 doppler_offset_lsb # 0.0001 Hz~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MeasEpochChannelType2>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MeasEpochChannelType2>))
  "Converts a ROS message object to a list"
  (cl:list 'MeasEpochChannelType2
    (cl:cons ':type (type msg))
    (cl:cons ':lock_time (lock_time msg))
    (cl:cons ':cn0 (cn0 msg))
    (cl:cons ':offsets_msb (offsets_msb msg))
    (cl:cons ':carrier_msb (carrier_msb msg))
    (cl:cons ':obs_info (obs_info msg))
    (cl:cons ':code_offset_lsb (code_offset_lsb msg))
    (cl:cons ':carrier_lsb (carrier_lsb msg))
    (cl:cons ':doppler_offset_lsb (doppler_offset_lsb msg))
))
