; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude VectorInfoCart.msg.html

(cl:defclass <VectorInfoCart> (roslisp-msg-protocol:ros-message)
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
   (delta_x
    :reader delta_x
    :initarg :delta_x
    :type cl:float
    :initform 0.0)
   (delta_y
    :reader delta_y
    :initarg :delta_y
    :type cl:float
    :initform 0.0)
   (delta_z
    :reader delta_z
    :initarg :delta_z
    :type cl:float
    :initform 0.0)
   (delta_vx
    :reader delta_vx
    :initarg :delta_vx
    :type cl:float
    :initform 0.0)
   (delta_vy
    :reader delta_vy
    :initarg :delta_vy
    :type cl:float
    :initform 0.0)
   (delta_vz
    :reader delta_vz
    :initarg :delta_vz
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

(cl:defclass VectorInfoCart (<VectorInfoCart>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VectorInfoCart>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VectorInfoCart)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<VectorInfoCart> is deprecated: use septentrio_gnss_driver-msg:VectorInfoCart instead.")))

(cl:ensure-generic-function 'nr_sv-val :lambda-list '(m))
(cl:defmethod nr_sv-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:nr_sv-val is deprecated.  Use septentrio_gnss_driver-msg:nr_sv instead.")
  (nr_sv m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:error-val is deprecated.  Use septentrio_gnss_driver-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:mode-val is deprecated.  Use septentrio_gnss_driver-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'misc-val :lambda-list '(m))
(cl:defmethod misc-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:misc-val is deprecated.  Use septentrio_gnss_driver-msg:misc instead.")
  (misc m))

(cl:ensure-generic-function 'delta_x-val :lambda-list '(m))
(cl:defmethod delta_x-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_x-val is deprecated.  Use septentrio_gnss_driver-msg:delta_x instead.")
  (delta_x m))

(cl:ensure-generic-function 'delta_y-val :lambda-list '(m))
(cl:defmethod delta_y-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_y-val is deprecated.  Use septentrio_gnss_driver-msg:delta_y instead.")
  (delta_y m))

(cl:ensure-generic-function 'delta_z-val :lambda-list '(m))
(cl:defmethod delta_z-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_z-val is deprecated.  Use septentrio_gnss_driver-msg:delta_z instead.")
  (delta_z m))

(cl:ensure-generic-function 'delta_vx-val :lambda-list '(m))
(cl:defmethod delta_vx-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_vx-val is deprecated.  Use septentrio_gnss_driver-msg:delta_vx instead.")
  (delta_vx m))

(cl:ensure-generic-function 'delta_vy-val :lambda-list '(m))
(cl:defmethod delta_vy-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_vy-val is deprecated.  Use septentrio_gnss_driver-msg:delta_vy instead.")
  (delta_vy m))

(cl:ensure-generic-function 'delta_vz-val :lambda-list '(m))
(cl:defmethod delta_vz-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:delta_vz-val is deprecated.  Use septentrio_gnss_driver-msg:delta_vz instead.")
  (delta_vz m))

(cl:ensure-generic-function 'azimuth-val :lambda-list '(m))
(cl:defmethod azimuth-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:azimuth-val is deprecated.  Use septentrio_gnss_driver-msg:azimuth instead.")
  (azimuth m))

(cl:ensure-generic-function 'elevation-val :lambda-list '(m))
(cl:defmethod elevation-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:elevation-val is deprecated.  Use septentrio_gnss_driver-msg:elevation instead.")
  (elevation m))

(cl:ensure-generic-function 'reference_id-val :lambda-list '(m))
(cl:defmethod reference_id-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:reference_id-val is deprecated.  Use septentrio_gnss_driver-msg:reference_id instead.")
  (reference_id m))

(cl:ensure-generic-function 'corr_age-val :lambda-list '(m))
(cl:defmethod corr_age-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:corr_age-val is deprecated.  Use septentrio_gnss_driver-msg:corr_age instead.")
  (corr_age m))

(cl:ensure-generic-function 'signal_info-val :lambda-list '(m))
(cl:defmethod signal_info-val ((m <VectorInfoCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:signal_info-val is deprecated.  Use septentrio_gnss_driver-msg:signal_info instead.")
  (signal_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VectorInfoCart>) ostream)
  "Serializes a message object of type '<VectorInfoCart>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nr_sv)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'misc)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'delta_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delta_vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delta_vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delta_vz))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VectorInfoCart>) istream)
  "Deserializes a message object of type '<VectorInfoCart>"
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
    (cl:setf (cl:slot-value msg 'delta_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_vx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_vy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_vz) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VectorInfoCart>)))
  "Returns string type for a message object of type '<VectorInfoCart>"
  "septentrio_gnss_driver/VectorInfoCart")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VectorInfoCart)))
  "Returns string type for a message object of type 'VectorInfoCart"
  "septentrio_gnss_driver/VectorInfoCart")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VectorInfoCart>)))
  "Returns md5sum for a message object of type '<VectorInfoCart>"
  "d072946ecd7f4319790a07cf534d27c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VectorInfoCart)))
  "Returns md5sum for a message object of type 'VectorInfoCart"
  "d072946ecd7f4319790a07cf534d27c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VectorInfoCart>)))
  "Returns full string definition for message of type '<VectorInfoCart>"
  (cl:format cl:nil "# VectorInfoCart block~%   ~%uint8   nr_sv~%uint8   error~%uint8   mode~%uint8   misc~%float64 delta_x      # m~%float64 delta_y      # m~%float64 delta_z      # m~%float32 delta_vx     # m~%float32 delta_vy     # m~%float32 delta_vz     # m~%uint16  azimuth      # 0.01 deg~%int16   elevation    # 0.01 deg~%uint16  reference_id~%uint16  corr_age     # 0.01 s~%uint32  signal_info~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VectorInfoCart)))
  "Returns full string definition for message of type 'VectorInfoCart"
  (cl:format cl:nil "# VectorInfoCart block~%   ~%uint8   nr_sv~%uint8   error~%uint8   mode~%uint8   misc~%float64 delta_x      # m~%float64 delta_y      # m~%float64 delta_z      # m~%float32 delta_vx     # m~%float32 delta_vy     # m~%float32 delta_vz     # m~%uint16  azimuth      # 0.01 deg~%int16   elevation    # 0.01 deg~%uint16  reference_id~%uint16  corr_age     # 0.01 s~%uint32  signal_info~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VectorInfoCart>))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VectorInfoCart>))
  "Converts a ROS message object to a list"
  (cl:list 'VectorInfoCart
    (cl:cons ':nr_sv (nr_sv msg))
    (cl:cons ':error (error msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':misc (misc msg))
    (cl:cons ':delta_x (delta_x msg))
    (cl:cons ':delta_y (delta_y msg))
    (cl:cons ':delta_z (delta_z msg))
    (cl:cons ':delta_vx (delta_vx msg))
    (cl:cons ':delta_vy (delta_vy msg))
    (cl:cons ':delta_vz (delta_vz msg))
    (cl:cons ':azimuth (azimuth msg))
    (cl:cons ':elevation (elevation msg))
    (cl:cons ':reference_id (reference_id msg))
    (cl:cons ':corr_age (corr_age msg))
    (cl:cons ':signal_info (signal_info msg))
))
