; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude INSNavCart.msg.html

(cl:defclass <INSNavCart> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (block_header
    :reader block_header
    :initarg :block_header
    :type septentrio_gnss_driver-msg:BlockHeader
    :initform (cl:make-instance 'septentrio_gnss_driver-msg:BlockHeader))
   (gnss_mode
    :reader gnss_mode
    :initarg :gnss_mode
    :type cl:fixnum
    :initform 0)
   (error
    :reader error
    :initarg :error
    :type cl:fixnum
    :initform 0)
   (info
    :reader info
    :initarg :info
    :type cl:fixnum
    :initform 0)
   (gnss_age
    :reader gnss_age
    :initarg :gnss_age
    :type cl:fixnum
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (accuracy
    :reader accuracy
    :initarg :accuracy
    :type cl:fixnum
    :initform 0)
   (latency
    :reader latency
    :initarg :latency
    :type cl:fixnum
    :initform 0)
   (datum
    :reader datum
    :initarg :datum
    :type cl:fixnum
    :initform 0)
   (sb_list
    :reader sb_list
    :initarg :sb_list
    :type cl:fixnum
    :initform 0)
   (x_std_dev
    :reader x_std_dev
    :initarg :x_std_dev
    :type cl:float
    :initform 0.0)
   (y_std_dev
    :reader y_std_dev
    :initarg :y_std_dev
    :type cl:float
    :initform 0.0)
   (z_std_dev
    :reader z_std_dev
    :initarg :z_std_dev
    :type cl:float
    :initform 0.0)
   (xy_cov
    :reader xy_cov
    :initarg :xy_cov
    :type cl:float
    :initform 0.0)
   (xz_cov
    :reader xz_cov
    :initarg :xz_cov
    :type cl:float
    :initform 0.0)
   (yz_cov
    :reader yz_cov
    :initarg :yz_cov
    :type cl:float
    :initform 0.0)
   (heading
    :reader heading
    :initarg :heading
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (heading_std_dev
    :reader heading_std_dev
    :initarg :heading_std_dev
    :type cl:float
    :initform 0.0)
   (pitch_std_dev
    :reader pitch_std_dev
    :initarg :pitch_std_dev
    :type cl:float
    :initform 0.0)
   (roll_std_dev
    :reader roll_std_dev
    :initarg :roll_std_dev
    :type cl:float
    :initform 0.0)
   (heading_pitch_cov
    :reader heading_pitch_cov
    :initarg :heading_pitch_cov
    :type cl:float
    :initform 0.0)
   (heading_roll_cov
    :reader heading_roll_cov
    :initarg :heading_roll_cov
    :type cl:float
    :initform 0.0)
   (pitch_roll_cov
    :reader pitch_roll_cov
    :initarg :pitch_roll_cov
    :type cl:float
    :initform 0.0)
   (vx
    :reader vx
    :initarg :vx
    :type cl:float
    :initform 0.0)
   (vy
    :reader vy
    :initarg :vy
    :type cl:float
    :initform 0.0)
   (vz
    :reader vz
    :initarg :vz
    :type cl:float
    :initform 0.0)
   (vx_std_dev
    :reader vx_std_dev
    :initarg :vx_std_dev
    :type cl:float
    :initform 0.0)
   (vy_std_dev
    :reader vy_std_dev
    :initarg :vy_std_dev
    :type cl:float
    :initform 0.0)
   (vz_std_dev
    :reader vz_std_dev
    :initarg :vz_std_dev
    :type cl:float
    :initform 0.0)
   (vx_vy_cov
    :reader vx_vy_cov
    :initarg :vx_vy_cov
    :type cl:float
    :initform 0.0)
   (vx_vz_cov
    :reader vx_vz_cov
    :initarg :vx_vz_cov
    :type cl:float
    :initform 0.0)
   (vy_vz_cov
    :reader vy_vz_cov
    :initarg :vy_vz_cov
    :type cl:float
    :initform 0.0))
)

(cl:defclass INSNavCart (<INSNavCart>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <INSNavCart>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'INSNavCart)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<INSNavCart> is deprecated: use septentrio_gnss_driver-msg:INSNavCart instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'gnss_mode-val :lambda-list '(m))
(cl:defmethod gnss_mode-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:gnss_mode-val is deprecated.  Use septentrio_gnss_driver-msg:gnss_mode instead.")
  (gnss_mode m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:error-val is deprecated.  Use septentrio_gnss_driver-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:info-val is deprecated.  Use septentrio_gnss_driver-msg:info instead.")
  (info m))

(cl:ensure-generic-function 'gnss_age-val :lambda-list '(m))
(cl:defmethod gnss_age-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:gnss_age-val is deprecated.  Use septentrio_gnss_driver-msg:gnss_age instead.")
  (gnss_age m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:x-val is deprecated.  Use septentrio_gnss_driver-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:y-val is deprecated.  Use septentrio_gnss_driver-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:z-val is deprecated.  Use septentrio_gnss_driver-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'accuracy-val :lambda-list '(m))
(cl:defmethod accuracy-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:accuracy-val is deprecated.  Use septentrio_gnss_driver-msg:accuracy instead.")
  (accuracy m))

(cl:ensure-generic-function 'latency-val :lambda-list '(m))
(cl:defmethod latency-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:latency-val is deprecated.  Use septentrio_gnss_driver-msg:latency instead.")
  (latency m))

(cl:ensure-generic-function 'datum-val :lambda-list '(m))
(cl:defmethod datum-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:datum-val is deprecated.  Use septentrio_gnss_driver-msg:datum instead.")
  (datum m))

(cl:ensure-generic-function 'sb_list-val :lambda-list '(m))
(cl:defmethod sb_list-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:sb_list-val is deprecated.  Use septentrio_gnss_driver-msg:sb_list instead.")
  (sb_list m))

(cl:ensure-generic-function 'x_std_dev-val :lambda-list '(m))
(cl:defmethod x_std_dev-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:x_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:x_std_dev instead.")
  (x_std_dev m))

(cl:ensure-generic-function 'y_std_dev-val :lambda-list '(m))
(cl:defmethod y_std_dev-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:y_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:y_std_dev instead.")
  (y_std_dev m))

(cl:ensure-generic-function 'z_std_dev-val :lambda-list '(m))
(cl:defmethod z_std_dev-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:z_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:z_std_dev instead.")
  (z_std_dev m))

(cl:ensure-generic-function 'xy_cov-val :lambda-list '(m))
(cl:defmethod xy_cov-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:xy_cov-val is deprecated.  Use septentrio_gnss_driver-msg:xy_cov instead.")
  (xy_cov m))

(cl:ensure-generic-function 'xz_cov-val :lambda-list '(m))
(cl:defmethod xz_cov-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:xz_cov-val is deprecated.  Use septentrio_gnss_driver-msg:xz_cov instead.")
  (xz_cov m))

(cl:ensure-generic-function 'yz_cov-val :lambda-list '(m))
(cl:defmethod yz_cov-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:yz_cov-val is deprecated.  Use septentrio_gnss_driver-msg:yz_cov instead.")
  (yz_cov m))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:heading-val is deprecated.  Use septentrio_gnss_driver-msg:heading instead.")
  (heading m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:pitch-val is deprecated.  Use septentrio_gnss_driver-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:roll-val is deprecated.  Use septentrio_gnss_driver-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'heading_std_dev-val :lambda-list '(m))
(cl:defmethod heading_std_dev-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:heading_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:heading_std_dev instead.")
  (heading_std_dev m))

(cl:ensure-generic-function 'pitch_std_dev-val :lambda-list '(m))
(cl:defmethod pitch_std_dev-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:pitch_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:pitch_std_dev instead.")
  (pitch_std_dev m))

(cl:ensure-generic-function 'roll_std_dev-val :lambda-list '(m))
(cl:defmethod roll_std_dev-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:roll_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:roll_std_dev instead.")
  (roll_std_dev m))

(cl:ensure-generic-function 'heading_pitch_cov-val :lambda-list '(m))
(cl:defmethod heading_pitch_cov-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:heading_pitch_cov-val is deprecated.  Use septentrio_gnss_driver-msg:heading_pitch_cov instead.")
  (heading_pitch_cov m))

(cl:ensure-generic-function 'heading_roll_cov-val :lambda-list '(m))
(cl:defmethod heading_roll_cov-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:heading_roll_cov-val is deprecated.  Use septentrio_gnss_driver-msg:heading_roll_cov instead.")
  (heading_roll_cov m))

(cl:ensure-generic-function 'pitch_roll_cov-val :lambda-list '(m))
(cl:defmethod pitch_roll_cov-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:pitch_roll_cov-val is deprecated.  Use septentrio_gnss_driver-msg:pitch_roll_cov instead.")
  (pitch_roll_cov m))

(cl:ensure-generic-function 'vx-val :lambda-list '(m))
(cl:defmethod vx-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vx-val is deprecated.  Use septentrio_gnss_driver-msg:vx instead.")
  (vx m))

(cl:ensure-generic-function 'vy-val :lambda-list '(m))
(cl:defmethod vy-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vy-val is deprecated.  Use septentrio_gnss_driver-msg:vy instead.")
  (vy m))

(cl:ensure-generic-function 'vz-val :lambda-list '(m))
(cl:defmethod vz-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vz-val is deprecated.  Use septentrio_gnss_driver-msg:vz instead.")
  (vz m))

(cl:ensure-generic-function 'vx_std_dev-val :lambda-list '(m))
(cl:defmethod vx_std_dev-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vx_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:vx_std_dev instead.")
  (vx_std_dev m))

(cl:ensure-generic-function 'vy_std_dev-val :lambda-list '(m))
(cl:defmethod vy_std_dev-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vy_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:vy_std_dev instead.")
  (vy_std_dev m))

(cl:ensure-generic-function 'vz_std_dev-val :lambda-list '(m))
(cl:defmethod vz_std_dev-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vz_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:vz_std_dev instead.")
  (vz_std_dev m))

(cl:ensure-generic-function 'vx_vy_cov-val :lambda-list '(m))
(cl:defmethod vx_vy_cov-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vx_vy_cov-val is deprecated.  Use septentrio_gnss_driver-msg:vx_vy_cov instead.")
  (vx_vy_cov m))

(cl:ensure-generic-function 'vx_vz_cov-val :lambda-list '(m))
(cl:defmethod vx_vz_cov-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vx_vz_cov-val is deprecated.  Use septentrio_gnss_driver-msg:vx_vz_cov instead.")
  (vx_vz_cov m))

(cl:ensure-generic-function 'vy_vz_cov-val :lambda-list '(m))
(cl:defmethod vy_vz_cov-val ((m <INSNavCart>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vy_vz_cov-val is deprecated.  Use septentrio_gnss_driver-msg:vy_vz_cov instead.")
  (vy_vz_cov m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <INSNavCart>) ostream)
  "Serializes a message object of type '<INSNavCart>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gnss_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gnss_age)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gnss_age)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'accuracy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'accuracy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'latency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'latency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'datum)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sb_list)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sb_list)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'xy_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'xz_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yz_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading_pitch_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading_roll_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_roll_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vx_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vy_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vz_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vx_vy_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vx_vz_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vy_vz_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <INSNavCart>) istream)
  "Deserializes a message object of type '<INSNavCart>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gnss_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gnss_age)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gnss_age)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'accuracy)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'accuracy)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'latency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'latency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'datum)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sb_list)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sb_list)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xy_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'xz_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yz_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_pitch_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_roll_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_roll_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vy_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vz_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx_vy_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vx_vz_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vy_vz_cov) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<INSNavCart>)))
  "Returns string type for a message object of type '<INSNavCart>"
  "septentrio_gnss_driver/INSNavCart")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'INSNavCart)))
  "Returns string type for a message object of type 'INSNavCart"
  "septentrio_gnss_driver/INSNavCart")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<INSNavCart>)))
  "Returns md5sum for a message object of type '<INSNavCart>"
  "ebb1a1e03ea3fac47ad5f19264160123")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'INSNavCart)))
  "Returns md5sum for a message object of type 'INSNavCart"
  "ebb1a1e03ea3fac47ad5f19264160123")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<INSNavCart>)))
  "Returns full string definition for message of type '<INSNavCart>"
  (cl:format cl:nil "# INSNavCart block ~%# Block_Number 4225~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header~%~%uint8       gnss_mode~%uint8       error~%uint16      info~%uint16      gnss_age # 0.01 s~%float64     x        # m~%float64     y        # m~%float64     z        # m~%uint16      accuracy # 0.01 m~%uint16      latency  # 0.1 ms~%uint8       datum    ~%#uint8       reserved~%uint16      sb_list~%~%# INSNavCartPosStdDev sub-block definition:~%# If the Position StdDev field is 1 then this sub block is available:~%float32     x_std_dev # m~%float32     y_std_dev # m~%float32     z_std_dev # m~%~%# INSNavCartPosCov sub-block definition:~%# If the Position Cov field is 1 then this sub block is available:~%float32     xy_cov # m^2~%float32     xz_cov # m^2~%float32     yz_cov # m^2~%~%# INSNavCartAtt sub-block definition:~%# If the Attitude field is 1 then this sub block is available:~%float32     heading # deg~%float32     pitch   # deg~%float32     roll    # deg~%~%# INSNavCartAttStdDev sub-block definition:~%# If the Attitude StdDev field is 1 then this sub block is available:~%float32     heading_std_dev # deg~%float32     pitch_std_dev   # deg~%float32     roll_std_dev    # deg~%~%# INSNavCartAttCov sub-block definition:~%# If the Attitude Cov field is 1 then this sub block is available:~%float32     heading_pitch_cov # deg^2~%float32     heading_roll_cov  # deg^2~%float32     pitch_roll_cov    # deg^2~%~%# INSNavCartVel sub-block definition:~%# If the Velocity field is 1 then this sub block is available:~%float32     vx # m/s~%float32     vy # m/S~%float32     vz # m/s~%~%# INSNavCartVelStdDev sub-block definition:~%# If the Velocity StdDev field is 1 then this sub block is available:~%float32     vx_std_dev # m/s~%float32     vy_std_dev # m/s~%float32     vz_std_dev # m/s~%~%# INSNavCartVelCov sub-block definition:~%# If the Velocity Cov field is 1 then this sub block is available:~%float32     vx_vy_cov # m^2/s^2~%float32     vx_vz_cov # m^2/s^2~%float32     vy_vz_cov # m^2/s^2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'INSNavCart)))
  "Returns full string definition for message of type 'INSNavCart"
  (cl:format cl:nil "# INSNavCart block ~%# Block_Number 4225~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header~%~%uint8       gnss_mode~%uint8       error~%uint16      info~%uint16      gnss_age # 0.01 s~%float64     x        # m~%float64     y        # m~%float64     z        # m~%uint16      accuracy # 0.01 m~%uint16      latency  # 0.1 ms~%uint8       datum    ~%#uint8       reserved~%uint16      sb_list~%~%# INSNavCartPosStdDev sub-block definition:~%# If the Position StdDev field is 1 then this sub block is available:~%float32     x_std_dev # m~%float32     y_std_dev # m~%float32     z_std_dev # m~%~%# INSNavCartPosCov sub-block definition:~%# If the Position Cov field is 1 then this sub block is available:~%float32     xy_cov # m^2~%float32     xz_cov # m^2~%float32     yz_cov # m^2~%~%# INSNavCartAtt sub-block definition:~%# If the Attitude field is 1 then this sub block is available:~%float32     heading # deg~%float32     pitch   # deg~%float32     roll    # deg~%~%# INSNavCartAttStdDev sub-block definition:~%# If the Attitude StdDev field is 1 then this sub block is available:~%float32     heading_std_dev # deg~%float32     pitch_std_dev   # deg~%float32     roll_std_dev    # deg~%~%# INSNavCartAttCov sub-block definition:~%# If the Attitude Cov field is 1 then this sub block is available:~%float32     heading_pitch_cov # deg^2~%float32     heading_roll_cov  # deg^2~%float32     pitch_roll_cov    # deg^2~%~%# INSNavCartVel sub-block definition:~%# If the Velocity field is 1 then this sub block is available:~%float32     vx # m/s~%float32     vy # m/S~%float32     vz # m/s~%~%# INSNavCartVelStdDev sub-block definition:~%# If the Velocity StdDev field is 1 then this sub block is available:~%float32     vx_std_dev # m/s~%float32     vy_std_dev # m/s~%float32     vz_std_dev # m/s~%~%# INSNavCartVelCov sub-block definition:~%# If the Velocity Cov field is 1 then this sub block is available:~%float32     vx_vy_cov # m^2/s^2~%float32     vx_vz_cov # m^2/s^2~%float32     vy_vz_cov # m^2/s^2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <INSNavCart>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'block_header))
     1
     1
     2
     2
     8
     8
     8
     2
     2
     1
     2
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <INSNavCart>))
  "Converts a ROS message object to a list"
  (cl:list 'INSNavCart
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':gnss_mode (gnss_mode msg))
    (cl:cons ':error (error msg))
    (cl:cons ':info (info msg))
    (cl:cons ':gnss_age (gnss_age msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':accuracy (accuracy msg))
    (cl:cons ':latency (latency msg))
    (cl:cons ':datum (datum msg))
    (cl:cons ':sb_list (sb_list msg))
    (cl:cons ':x_std_dev (x_std_dev msg))
    (cl:cons ':y_std_dev (y_std_dev msg))
    (cl:cons ':z_std_dev (z_std_dev msg))
    (cl:cons ':xy_cov (xy_cov msg))
    (cl:cons ':xz_cov (xz_cov msg))
    (cl:cons ':yz_cov (yz_cov msg))
    (cl:cons ':heading (heading msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':heading_std_dev (heading_std_dev msg))
    (cl:cons ':pitch_std_dev (pitch_std_dev msg))
    (cl:cons ':roll_std_dev (roll_std_dev msg))
    (cl:cons ':heading_pitch_cov (heading_pitch_cov msg))
    (cl:cons ':heading_roll_cov (heading_roll_cov msg))
    (cl:cons ':pitch_roll_cov (pitch_roll_cov msg))
    (cl:cons ':vx (vx msg))
    (cl:cons ':vy (vy msg))
    (cl:cons ':vz (vz msg))
    (cl:cons ':vx_std_dev (vx_std_dev msg))
    (cl:cons ':vy_std_dev (vy_std_dev msg))
    (cl:cons ':vz_std_dev (vz_std_dev msg))
    (cl:cons ':vx_vy_cov (vx_vy_cov msg))
    (cl:cons ':vx_vz_cov (vx_vz_cov msg))
    (cl:cons ':vy_vz_cov (vy_vz_cov msg))
))
