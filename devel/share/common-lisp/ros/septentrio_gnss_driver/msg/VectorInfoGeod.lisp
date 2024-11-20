; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude VectorInfoGeod.msg.html

(cl:defclass <VectorInfoGeod> (roslisp-msg-protocol:ros-message)
  ((nr_sv
    :reader nr_sv
    :initarg :nr_sv
    :type cl:fixnum
    :initform 0)
   (error
    :reader error
    :initarg :error
    :type cl:fixnum
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (misc
    :reader misc
    :initarg :misc
    :type cl:fixnum
    :initform 0)
   (delta_east
    :reader delta_east
    :initarg :delta_east
    :type cl:float
    :initform 0.0)
   (delta_north
    :reader delta_north
    :initarg :delta_north
    :type cl:float
    :initform 0.0)
   (delta_up
    :reader delta_up
    :initarg :delta_up
    :type cl:float
    :initform 0.0)
   (delta_ve
    :reader delta_ve
    :initarg :delta_ve
    :type cl:float
    :initform 0.0)
   (delta_vn
    :reader delta_vn
    :initarg :delta_vn
    :type cl:float
    :initform 0.0)
   (delta_vu
    :reader delta_vu
    :initarg :delta_vu
    :type cl:float
    :initform 0.0)
   (azimuth
    :reader azimuth
    :initarg :azimuth
    :type cl:fixnum
    :initform 0)
   (elevation
    :reader elevation
    :initarg :elevation
    :type cl:fixnum
    :initform 0)
   (reference_id
    :reader reference_id
    :initarg :reference_id
    :type cl:fixnum
    :initform 0)
   (corr_age
    :reader corr_age
    :initarg :corr_age
    :type cl:fixnum
    :initform 0)
   (signal_info
    :reader signal_info
    :initarg :signal_info
    :type cl:integer
    :initform 0))
)

(cl:defclass VectorInfoGeod (<VectorInfoGeod>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VectorInfoGeod>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VectorInfoGeod)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<VectorInfoGeod> is deprecated: use septentrio_gnss_driver-msg:VectorInfoGeod instead.")))

(cl:ensure-generic-function 'nr_sv-val :lambda-list '(m))
(cl:defmethod nr_sv-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:nr_sv-val is deprecated.  Use septentrio_gnss_driver-msg:nr_sv instead.")
  (nr_sv m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:error-val is deprecated.  Use septentrio_gnss_driver-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:mode-val is deprecated.  Use septentrio_gnss_driver-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'misc-val :lambda-list '(m))
(cl:defmethod misc-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:misc-val is deprecated.  Use septentrio_gnss_driver-msg:misc instead.")
  (misc m))

(cl:ensure-generic-function 'delta_east-val :lambda-list '(m))
(cl:defmethod delta_east-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_east-val is deprecated.  Use septentrio_gnss_driver-msg:delta_east instead.")
  (delta_east m))

(cl:ensure-generic-function 'delta_north-val :lambda-list '(m))
(cl:defmethod delta_north-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_north-val is deprecated.  Use septentrio_gnss_driver-msg:delta_north instead.")
  (delta_north m))

(cl:ensure-generic-function 'delta_up-val :lambda-list '(m))
(cl:defmethod delta_up-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_up-val is deprecated.  Use septentrio_gnss_driver-msg:delta_up instead.")
  (delta_up m))

(cl:ensure-generic-function 'delta_ve-val :lambda-list '(m))
(cl:defmethod delta_ve-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_ve-val is deprecated.  Use septentrio_gnss_driver-msg:delta_ve instead.")
  (delta_ve m))

(cl:ensure-generic-function 'delta_vn-val :lambda-list '(m))
(cl:defmethod delta_vn-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_vn-val is deprecated.  Use septentrio_gnss_driver-msg:delta_vn instead.")
  (delta_vn m))

(cl:ensure-generic-function 'delta_vu-val :lambda-list '(m))
(cl:defmethod delta_vu-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_vu-val is deprecated.  Use septentrio_gnss_driver-msg:delta_vu instead.")
  (delta_vu m))

(cl:ensure-generic-function 'azimuth-val :lambda-list '(m))
(cl:defmethod azimuth-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:azimuth-val is deprecated.  Use septentrio_gnss_driver-msg:azimuth instead.")
  (azimuth m))

(cl:ensure-generic-function 'elevation-val :lambda-list '(m))
(cl:defmethod elevation-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:elevation-val is deprecated.  Use septentrio_gnss_driver-msg:elevation instead.")
  (elevation m))

(cl:ensure-generic-function 'reference_id-val :lambda-list '(m))
(cl:defmethod reference_id-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:reference_id-val is deprecated.  Use septentrio_gnss_driver-msg:reference_id instead.")
  (reference_id m))

(cl:ensure-generic-function 'corr_age-val :lambda-list '(m))
(cl:defmethod corr_age-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:corr_age-val is deprecated.  Use septentrio_gnss_driver-msg:corr_age instead.")
  (corr_age m))

(cl:ensure-generic-function 'signal_info-val :lambda-list '(m))
(cl:defmethod signal_info-val ((m <VectorInfoGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:signal_info-val is deprecated.  Use septentrio_gnss_driver-msg:signal_info instead.")
  (signal_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VectorInfoGeod>) ostream)
  "Serializes a message object of type '<VectorInfoGeod>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nr_sv)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'misc)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_east))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_north))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_up))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delta_ve))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delta_vn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delta_vu))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'azimuth)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'azimuth)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'elevation)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'corr_age)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'corr_age)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'signal_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'signal_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'signal_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'signal_info)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VectorInfoGeod>) istream)
  "Deserializes a message object of type '<VectorInfoGeod>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nr_sv)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'misc)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_east) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_north) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_up) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_ve) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_vn) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_vu) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'azimuth)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'azimuth)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'elevation) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'corr_age)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'corr_age)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'signal_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'signal_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'signal_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'signal_info)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VectorInfoGeod>)))
  "Returns string type for a message object of type '<VectorInfoGeod>"
  "septentrio_gnss_driver/VectorInfoGeod")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VectorInfoGeod)))
  "Returns string type for a message object of type 'VectorInfoGeod"
  "septentrio_gnss_driver/VectorInfoGeod")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VectorInfoGeod>)))
  "Returns md5sum for a message object of type '<VectorInfoGeod>"
  "dbd3c8ede9fdca23023a59b11ba2b072")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VectorInfoGeod)))
  "Returns md5sum for a message object of type 'VectorInfoGeod"
  "dbd3c8ede9fdca23023a59b11ba2b072")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VectorInfoGeod>)))
  "Returns full string definition for message of type '<VectorInfoGeod>"
  (cl:format cl:nil "# VectorInfoGeod block~%   ~%uint8   nr_sv~%uint8   error~%uint8   mode~%uint8   misc~%float64 delta_east   # m~%float64 delta_north  # m~%float64 delta_up     # m~%float32 delta_ve     # m~%float32 delta_vn     # m~%float32 delta_vu     # m~%uint16  azimuth      # 0.01 deg~%int16   elevation    # 0.01 deg~%uint16  reference_id~%uint16  corr_age     # 0.01 s~%uint32  signal_info~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VectorInfoGeod)))
  "Returns full string definition for message of type 'VectorInfoGeod"
  (cl:format cl:nil "# VectorInfoGeod block~%   ~%uint8   nr_sv~%uint8   error~%uint8   mode~%uint8   misc~%float64 delta_east   # m~%float64 delta_north  # m~%float64 delta_up     # m~%float32 delta_ve     # m~%float32 delta_vn     # m~%float32 delta_vu     # m~%uint16  azimuth      # 0.01 deg~%int16   elevation    # 0.01 deg~%uint16  reference_id~%uint16  corr_age     # 0.01 s~%uint32  signal_info~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VectorInfoGeod>))
  (cl:+ 0
     1
     1
     1
     1
     8
     8
     8
     4
     4
     4
     2
     2
     2
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VectorInfoGeod>))
  "Converts a ROS message object to a list"
  (cl:list 'VectorInfoGeod
    (cl:cons ':nr_sv (nr_sv msg))
    (cl:cons ':error (error msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':misc (misc msg))
    (cl:cons ':delta_east (delta_east msg))
    (cl:cons ':delta_north (delta_north msg))
    (cl:cons ':delta_up (delta_up msg))
    (cl:cons ':delta_ve (delta_ve msg))
    (cl:cons ':delta_vn (delta_vn msg))
    (cl:cons ':delta_vu (delta_vu msg))
    (cl:cons ':azimuth (azimuth msg))
    (cl:cons ':elevation (elevation msg))
    (cl:cons ':reference_id (reference_id msg))
    (cl:cons ':corr_age (corr_age msg))
    (cl:cons ':signal_info (signal_info msg))
))
