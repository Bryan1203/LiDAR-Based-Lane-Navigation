; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude MeasEpochChannelType1.msg.html

(cl:defclass <MeasEpochChannelType1> (roslisp-msg-protocol:ros-message)
  ((rx_channel
    :reader rx_channel
    :initarg :rx_channel
    :type cl:fixnum
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (sv_id
    :reader sv_id
    :initarg :sv_id
    :type cl:fixnum
    :initform 0)
   (misc
    :reader misc
    :initarg :misc
    :type cl:fixnum
    :initform 0)
   (code_lsb
    :reader code_lsb
    :initarg :code_lsb
    :type cl:integer
    :initform 0)
   (doppler
    :reader doppler
    :initarg :doppler
    :type cl:integer
    :initform 0)
   (carrier_lsb
    :reader carrier_lsb
    :initarg :carrier_lsb
    :type cl:fixnum
    :initform 0)
   (carrier_msb
    :reader carrier_msb
    :initarg :carrier_msb
    :type cl:fixnum
    :initform 0)
   (cn0
    :reader cn0
    :initarg :cn0
    :type cl:fixnum
    :initform 0)
   (lock_time
    :reader lock_time
    :initarg :lock_time
    :type cl:fixnum
    :initform 0)
   (obs_info
    :reader obs_info
    :initarg :obs_info
    :type cl:fixnum
    :initform 0)
   (n2
    :reader n2
    :initarg :n2
    :type cl:fixnum
    :initform 0)
   (type2
    :reader type2
    :initarg :type2
    :type (cl:vector septentrio_gnss_driver-msg:MeasEpochChannelType2)
   :initform (cl:make-array 0 :element-type 'septentrio_gnss_driver-msg:MeasEpochChannelType2 :initial-element (cl:make-instance 'septentrio_gnss_driver-msg:MeasEpochChannelType2))))
)

(cl:defclass MeasEpochChannelType1 (<MeasEpochChannelType1>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MeasEpochChannelType1>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MeasEpochChannelType1)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<MeasEpochChannelType1> is deprecated: use septentrio_gnss_driver-msg:MeasEpochChannelType1 instead.")))

(cl:ensure-generic-function 'rx_channel-val :lambda-list '(m))
(cl:defmethod rx_channel-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:rx_channel-val is deprecated.  Use septentrio_gnss_driver-msg:rx_channel instead.")
  (rx_channel m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:type-val is deprecated.  Use septentrio_gnss_driver-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'sv_id-val :lambda-list '(m))
(cl:defmethod sv_id-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:sv_id-val is deprecated.  Use septentrio_gnss_driver-msg:sv_id instead.")
  (sv_id m))

(cl:ensure-generic-function 'misc-val :lambda-list '(m))
(cl:defmethod misc-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:misc-val is deprecated.  Use septentrio_gnss_driver-msg:misc instead.")
  (misc m))

(cl:ensure-generic-function 'code_lsb-val :lambda-list '(m))
(cl:defmethod code_lsb-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:code_lsb-val is deprecated.  Use septentrio_gnss_driver-msg:code_lsb instead.")
  (code_lsb m))

(cl:ensure-generic-function 'doppler-val :lambda-list '(m))
(cl:defmethod doppler-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:doppler-val is deprecated.  Use septentrio_gnss_driver-msg:doppler instead.")
  (doppler m))

(cl:ensure-generic-function 'carrier_lsb-val :lambda-list '(m))
(cl:defmethod carrier_lsb-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:carrier_lsb-val is deprecated.  Use septentrio_gnss_driver-msg:carrier_lsb instead.")
  (carrier_lsb m))

(cl:ensure-generic-function 'carrier_msb-val :lambda-list '(m))
(cl:defmethod carrier_msb-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:carrier_msb-val is deprecated.  Use septentrio_gnss_driver-msg:carrier_msb instead.")
  (carrier_msb m))

(cl:ensure-generic-function 'cn0-val :lambda-list '(m))
(cl:defmethod cn0-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cn0-val is deprecated.  Use septentrio_gnss_driver-msg:cn0 instead.")
  (cn0 m))

(cl:ensure-generic-function 'lock_time-val :lambda-list '(m))
(cl:defmethod lock_time-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:lock_time-val is deprecated.  Use septentrio_gnss_driver-msg:lock_time instead.")
  (lock_time m))

(cl:ensure-generic-function 'obs_info-val :lambda-list '(m))
(cl:defmethod obs_info-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:obs_info-val is deprecated.  Use septentrio_gnss_driver-msg:obs_info instead.")
  (obs_info m))

(cl:ensure-generic-function 'n2-val :lambda-list '(m))
(cl:defmethod n2-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:n2-val is deprecated.  Use septentrio_gnss_driver-msg:n2 instead.")
  (n2 m))

(cl:ensure-generic-function 'type2-val :lambda-list '(m))
(cl:defmethod type2-val ((m <MeasEpochChannelType1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:type2-val is deprecated.  Use septentrio_gnss_driver-msg:type2 instead.")
  (type2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MeasEpochChannelType1>) ostream)
  "Serializes a message object of type '<MeasEpochChannelType1>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rx_channel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sv_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'misc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'code_lsb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'code_lsb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'code_lsb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'code_lsb)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'doppler)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'carrier_lsb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'carrier_lsb)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'carrier_msb)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cn0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lock_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lock_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obs_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n2)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'type2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'type2))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MeasEpochChannelType1>) istream)
  "Deserializes a message object of type '<MeasEpochChannelType1>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rx_channel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sv_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'misc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'code_lsb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'code_lsb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'code_lsb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'code_lsb)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'doppler) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'carrier_lsb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'carrier_lsb)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'carrier_msb) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cn0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lock_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lock_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obs_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n2)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'type2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'type2)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'septentrio_gnss_driver-msg:MeasEpochChannelType2))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MeasEpochChannelType1>)))
  "Returns string type for a message object of type '<MeasEpochChannelType1>"
  "septentrio_gnss_driver/MeasEpochChannelType1")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MeasEpochChannelType1)))
  "Returns string type for a message object of type 'MeasEpochChannelType1"
  "septentrio_gnss_driver/MeasEpochChannelType1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MeasEpochChannelType1>)))
  "Returns md5sum for a message object of type '<MeasEpochChannelType1>"
  "6ec5531728aded72fdbf7e7550cabd87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MeasEpochChannelType1)))
  "Returns md5sum for a message object of type 'MeasEpochChannelType1"
  "6ec5531728aded72fdbf7e7550cabd87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MeasEpochChannelType1>)))
  "Returns full string definition for message of type '<MeasEpochChannelType1>"
  (cl:format cl:nil "# MeasEpochChannelType1 block~%   ~%uint8  rx_channel~%uint8  type~%uint8  sv_id~%uint8  misc        # 4294967.296 m~%uint32 code_lsb    # 0.001 m~%int32  doppler     # 0.0001 Hz~%uint16 carrier_lsb # 0.001 cycles~%int8   carrier_msb # 65.536 cycles~%uint8  cn0         # 0.25 dB-Hz~%uint16 lock_time~%uint8  obs_info~%uint8  n2~%~%MeasEpochChannelType2[] type2~%================================================================================~%MSG: septentrio_gnss_driver/MeasEpochChannelType2~%# MeasEpochChannelType2 block~%   ~%uint8  type~%uint8  lock_time          # s~%uint8  cn0                # 0.25 dB-Hz~%uint8  offsets_msb        # 65.536 m or 65.536 Hz~%int8   carrier_msb        # 65.536 cycles~%uint8  obs_info~%uint16 code_offset_lsb    # 0.001 m~%uint16 carrier_lsb        # 0.001 cycles~%uint16 doppler_offset_lsb # 0.0001 Hz~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MeasEpochChannelType1)))
  "Returns full string definition for message of type 'MeasEpochChannelType1"
  (cl:format cl:nil "# MeasEpochChannelType1 block~%   ~%uint8  rx_channel~%uint8  type~%uint8  sv_id~%uint8  misc        # 4294967.296 m~%uint32 code_lsb    # 0.001 m~%int32  doppler     # 0.0001 Hz~%uint16 carrier_lsb # 0.001 cycles~%int8   carrier_msb # 65.536 cycles~%uint8  cn0         # 0.25 dB-Hz~%uint16 lock_time~%uint8  obs_info~%uint8  n2~%~%MeasEpochChannelType2[] type2~%================================================================================~%MSG: septentrio_gnss_driver/MeasEpochChannelType2~%# MeasEpochChannelType2 block~%   ~%uint8  type~%uint8  lock_time          # s~%uint8  cn0                # 0.25 dB-Hz~%uint8  offsets_msb        # 65.536 m or 65.536 Hz~%int8   carrier_msb        # 65.536 cycles~%uint8  obs_info~%uint16 code_offset_lsb    # 0.001 m~%uint16 carrier_lsb        # 0.001 cycles~%uint16 doppler_offset_lsb # 0.0001 Hz~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MeasEpochChannelType1>))
  (cl:+ 0
     1
     1
     1
     1
     4
     4
     2
     1
     1
     2
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'type2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MeasEpochChannelType1>))
  "Converts a ROS message object to a list"
  (cl:list 'MeasEpochChannelType1
    (cl:cons ':rx_channel (rx_channel msg))
    (cl:cons ':type (type msg))
    (cl:cons ':sv_id (sv_id msg))
    (cl:cons ':misc (misc msg))
    (cl:cons ':code_lsb (code_lsb msg))
    (cl:cons ':doppler (doppler msg))
    (cl:cons ':carrier_lsb (carrier_lsb msg))
    (cl:cons ':carrier_msb (carrier_msb msg))
    (cl:cons ':cn0 (cn0 msg))
    (cl:cons ':lock_time (lock_time msg))
    (cl:cons ':obs_info (obs_info msg))
    (cl:cons ':n2 (n2 msg))
    (cl:cons ':type2 (type2 msg))
))
