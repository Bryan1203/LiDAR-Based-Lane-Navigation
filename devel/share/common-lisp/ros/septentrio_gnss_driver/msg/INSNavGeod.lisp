; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude INSNavGeod.msg.html

(cl:defclass <INSNavGeod> (roslisp-msg-protocol:ros-message)
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
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0)
   (undulation
    :reader undulation
    :initarg :undulation
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
   (latitude_std_dev
    :reader latitude_std_dev
    :initarg :latitude_std_dev
    :type cl:float
    :initform 0.0)
   (longitude_std_dev
    :reader longitude_std_dev
    :initarg :longitude_std_dev
    :type cl:float
    :initform 0.0)
   (height_std_dev
    :reader height_std_dev
    :initarg :height_std_dev
    :type cl:float
    :initform 0.0)
   (latitude_longitude_cov
    :reader latitude_longitude_cov
    :initarg :latitude_longitude_cov
    :type cl:float
    :initform 0.0)
   (latitude_height_cov
    :reader latitude_height_cov
    :initarg :latitude_height_cov
    :type cl:float
    :initform 0.0)
   (longitude_height_cov
    :reader longitude_height_cov
    :initarg :longitude_height_cov
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
   (ve
    :reader ve
    :initarg :ve
    :type cl:float
    :initform 0.0)
   (vn
    :reader vn
    :initarg :vn
    :type cl:float
    :initform 0.0)
   (vu
    :reader vu
    :initarg :vu
    :type cl:float
    :initform 0.0)
   (ve_std_dev
    :reader ve_std_dev
    :initarg :ve_std_dev
    :type cl:float
    :initform 0.0)
   (vn_std_dev
    :reader vn_std_dev
    :initarg :vn_std_dev
    :type cl:float
    :initform 0.0)
   (vu_std_dev
    :reader vu_std_dev
    :initarg :vu_std_dev
    :type cl:float
    :initform 0.0)
   (ve_vn_cov
    :reader ve_vn_cov
    :initarg :ve_vn_cov
    :type cl:float
    :initform 0.0)
   (ve_vu_cov
    :reader ve_vu_cov
    :initarg :ve_vu_cov
    :type cl:float
    :initform 0.0)
   (vn_vu_cov
    :reader vn_vu_cov
    :initarg :vn_vu_cov
    :type cl:float
    :initform 0.0))
)

(cl:defclass INSNavGeod (<INSNavGeod>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <INSNavGeod>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'INSNavGeod)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<INSNavGeod> is deprecated: use septentrio_gnss_driver-msg:INSNavGeod instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'gnss_mode-val :lambda-list '(m))
(cl:defmethod gnss_mode-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:gnss_mode-val is deprecated.  Use septentrio_gnss_driver-msg:gnss_mode instead.")
  (gnss_mode m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:error-val is deprecated.  Use septentrio_gnss_driver-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'info-val :lambda-list '(m))
(cl:defmethod info-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:info-val is deprecated.  Use septentrio_gnss_driver-msg:info instead.")
  (info m))

(cl:ensure-generic-function 'gnss_age-val :lambda-list '(m))
(cl:defmethod gnss_age-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:gnss_age-val is deprecated.  Use septentrio_gnss_driver-msg:gnss_age instead.")
  (gnss_age m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:latitude-val is deprecated.  Use septentrio_gnss_driver-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:longitude-val is deprecated.  Use septentrio_gnss_driver-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:height-val is deprecated.  Use septentrio_gnss_driver-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'undulation-val :lambda-list '(m))
(cl:defmethod undulation-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:undulation-val is deprecated.  Use septentrio_gnss_driver-msg:undulation instead.")
  (undulation m))

(cl:ensure-generic-function 'accuracy-val :lambda-list '(m))
(cl:defmethod accuracy-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:accuracy-val is deprecated.  Use septentrio_gnss_driver-msg:accuracy instead.")
  (accuracy m))

(cl:ensure-generic-function 'latency-val :lambda-list '(m))
(cl:defmethod latency-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:latency-val is deprecated.  Use septentrio_gnss_driver-msg:latency instead.")
  (latency m))

(cl:ensure-generic-function 'datum-val :lambda-list '(m))
(cl:defmethod datum-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:datum-val is deprecated.  Use septentrio_gnss_driver-msg:datum instead.")
  (datum m))

(cl:ensure-generic-function 'sb_list-val :lambda-list '(m))
(cl:defmethod sb_list-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:sb_list-val is deprecated.  Use septentrio_gnss_driver-msg:sb_list instead.")
  (sb_list m))

(cl:ensure-generic-function 'latitude_std_dev-val :lambda-list '(m))
(cl:defmethod latitude_std_dev-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:latitude_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:latitude_std_dev instead.")
  (latitude_std_dev m))

(cl:ensure-generic-function 'longitude_std_dev-val :lambda-list '(m))
(cl:defmethod longitude_std_dev-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:longitude_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:longitude_std_dev instead.")
  (longitude_std_dev m))

(cl:ensure-generic-function 'height_std_dev-val :lambda-list '(m))
(cl:defmethod height_std_dev-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:height_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:height_std_dev instead.")
  (height_std_dev m))

(cl:ensure-generic-function 'latitude_longitude_cov-val :lambda-list '(m))
(cl:defmethod latitude_longitude_cov-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:latitude_longitude_cov-val is deprecated.  Use septentrio_gnss_driver-msg:latitude_longitude_cov instead.")
  (latitude_longitude_cov m))

(cl:ensure-generic-function 'latitude_height_cov-val :lambda-list '(m))
(cl:defmethod latitude_height_cov-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:latitude_height_cov-val is deprecated.  Use septentrio_gnss_driver-msg:latitude_height_cov instead.")
  (latitude_height_cov m))

(cl:ensure-generic-function 'longitude_height_cov-val :lambda-list '(m))
(cl:defmethod longitude_height_cov-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:longitude_height_cov-val is deprecated.  Use septentrio_gnss_driver-msg:longitude_height_cov instead.")
  (longitude_height_cov m))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:heading-val is deprecated.  Use septentrio_gnss_driver-msg:heading instead.")
  (heading m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:pitch-val is deprecated.  Use septentrio_gnss_driver-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:roll-val is deprecated.  Use septentrio_gnss_driver-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'heading_std_dev-val :lambda-list '(m))
(cl:defmethod heading_std_dev-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:heading_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:heading_std_dev instead.")
  (heading_std_dev m))

(cl:ensure-generic-function 'pitch_std_dev-val :lambda-list '(m))
(cl:defmethod pitch_std_dev-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:pitch_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:pitch_std_dev instead.")
  (pitch_std_dev m))

(cl:ensure-generic-function 'roll_std_dev-val :lambda-list '(m))
(cl:defmethod roll_std_dev-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:roll_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:roll_std_dev instead.")
  (roll_std_dev m))

(cl:ensure-generic-function 'heading_pitch_cov-val :lambda-list '(m))
(cl:defmethod heading_pitch_cov-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:heading_pitch_cov-val is deprecated.  Use septentrio_gnss_driver-msg:heading_pitch_cov instead.")
  (heading_pitch_cov m))

(cl:ensure-generic-function 'heading_roll_cov-val :lambda-list '(m))
(cl:defmethod heading_roll_cov-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:heading_roll_cov-val is deprecated.  Use septentrio_gnss_driver-msg:heading_roll_cov instead.")
  (heading_roll_cov m))

(cl:ensure-generic-function 'pitch_roll_cov-val :lambda-list '(m))
(cl:defmethod pitch_roll_cov-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:pitch_roll_cov-val is deprecated.  Use septentrio_gnss_driver-msg:pitch_roll_cov instead.")
  (pitch_roll_cov m))

(cl:ensure-generic-function 've-val :lambda-list '(m))
(cl:defmethod ve-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:ve-val is deprecated.  Use septentrio_gnss_driver-msg:ve instead.")
  (ve m))

(cl:ensure-generic-function 'vn-val :lambda-list '(m))
(cl:defmethod vn-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vn-val is deprecated.  Use septentrio_gnss_driver-msg:vn instead.")
  (vn m))

(cl:ensure-generic-function 'vu-val :lambda-list '(m))
(cl:defmethod vu-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vu-val is deprecated.  Use septentrio_gnss_driver-msg:vu instead.")
  (vu m))

(cl:ensure-generic-function 've_std_dev-val :lambda-list '(m))
(cl:defmethod ve_std_dev-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:ve_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:ve_std_dev instead.")
  (ve_std_dev m))

(cl:ensure-generic-function 'vn_std_dev-val :lambda-list '(m))
(cl:defmethod vn_std_dev-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vn_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:vn_std_dev instead.")
  (vn_std_dev m))

(cl:ensure-generic-function 'vu_std_dev-val :lambda-list '(m))
(cl:defmethod vu_std_dev-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vu_std_dev-val is deprecated.  Use septentrio_gnss_driver-msg:vu_std_dev instead.")
  (vu_std_dev m))

(cl:ensure-generic-function 've_vn_cov-val :lambda-list '(m))
(cl:defmethod ve_vn_cov-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:ve_vn_cov-val is deprecated.  Use septentrio_gnss_driver-msg:ve_vn_cov instead.")
  (ve_vn_cov m))

(cl:ensure-generic-function 've_vu_cov-val :lambda-list '(m))
(cl:defmethod ve_vu_cov-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:ve_vu_cov-val is deprecated.  Use septentrio_gnss_driver-msg:ve_vu_cov instead.")
  (ve_vu_cov m))

(cl:ensure-generic-function 'vn_vu_cov-val :lambda-list '(m))
(cl:defmethod vn_vu_cov-val ((m <INSNavGeod>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vn_vu_cov-val is deprecated.  Use septentrio_gnss_driver-msg:vn_vu_cov instead.")
  (vn_vu_cov m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <INSNavGeod>) ostream)
  "Serializes a message object of type '<INSNavGeod>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gnss_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gnss_age)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gnss_age)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'undulation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'accuracy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'accuracy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'latency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'latency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'datum)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sb_list)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sb_list)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'longitude_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude_longitude_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude_height_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'longitude_height_cov))))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 've))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vu))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 've_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vn_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vu_std_dev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 've_vn_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 've_vu_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vn_vu_cov))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <INSNavGeod>) istream)
  "Deserializes a message object of type '<INSNavGeod>"
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
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'undulation) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'latitude_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude_longitude_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude_height_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude_height_cov) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 've) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vn) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vu) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 've_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vn_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vu_std_dev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 've_vn_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 've_vu_cov) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vn_vu_cov) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<INSNavGeod>)))
  "Returns string type for a message object of type '<INSNavGeod>"
  "septentrio_gnss_driver/INSNavGeod")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'INSNavGeod)))
  "Returns string type for a message object of type 'INSNavGeod"
  "septentrio_gnss_driver/INSNavGeod")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<INSNavGeod>)))
  "Returns md5sum for a message object of type '<INSNavGeod>"
  "7e95cd9c24ef355a547323f4ca3d6e14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'INSNavGeod)))
  "Returns md5sum for a message object of type 'INSNavGeod"
  "7e95cd9c24ef355a547323f4ca3d6e14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<INSNavGeod>)))
  "Returns full string definition for message of type '<INSNavGeod>"
  (cl:format cl:nil "# INSNavGeod block ~%# Block_Number 4226~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header~%~%uint8       gnss_mode~%uint8       error~%uint16      info~%uint16      gnss_age   # 0.01 s~%float64     latitude   # rad~%float64     longitude  # rad~%float64     height     # m (ellipsoidal)~%float32     undulation # m~%uint16      accuracy   # 0.01 m~%uint16      latency    # 0.0001 s~%uint8       datum~%#uint8       reserved~%uint16      sb_list~%~%# INSNavGeodPosStdDev sub-block definition:~%# If the Position StdDev field is 1 then this sub block is available:~%float32     latitude_std_dev  # m~%float32     longitude_std_dev # m~%float32     height_std_dev    # m~%~%# INSNavGeodPosCov sub-block definition:~%# If the Position Cov field is 1 then this sub block is available:~%float32     latitude_longitude_cov # m^2~%float32     latitude_height_cov    # m^2~%float32     longitude_height_cov   # m^2~%~%# INSNavGeodAtt sub-block definition:~%# If the Attitude field is 1 then this sub block is available:~%float32     heading # deg~%float32     pitch   # deg~%float32     roll    # deg~%~%# INSNavGeodAttStdDev sub-block definition:~%# If the Attitude StdDev field is 1 then this sub block is available:~%float32     heading_std_dev # deg~%float32     pitch_std_dev   # deg~%float32     roll_std_dev    # deg~%~%# INSNavGeodAttCov sub-block definition:~%# If the Attitude Cov field is 1 then this sub block is available:~%float32     heading_pitch_cov # deg^2~%float32     heading_roll_cov  # deg^2~%float32     pitch_roll_cov    # deg^2~%~%# INSNavGeodVel sub-block definition:~%# If the Velocity field is 1 then this sub block is available:~%float32     ve # m/s~%float32     vn # m/s ~%float32     vu # m/s~%~%# INSNavGeodVelStdDev sub-block definition:~%# If the Velocity StdDev field is 1 then this sub block is available:~%float32     ve_std_dev # m/s~%float32     vn_std_dev # m/s~%float32     vu_std_dev # m/s~%~%# INSNavGeodVelCov sub-block definition:~%# If the Velocity Cov field is 1 then this sub block is available:~%float32     ve_vn_cov # m^2/s^2~%float32     ve_vu_cov # m^2/s^2~%float32     vn_vu_cov # m^2/s^2   ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'INSNavGeod)))
  "Returns full string definition for message of type 'INSNavGeod"
  (cl:format cl:nil "# INSNavGeod block ~%# Block_Number 4226~%~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader block_header~%~%uint8       gnss_mode~%uint8       error~%uint16      info~%uint16      gnss_age   # 0.01 s~%float64     latitude   # rad~%float64     longitude  # rad~%float64     height     # m (ellipsoidal)~%float32     undulation # m~%uint16      accuracy   # 0.01 m~%uint16      latency    # 0.0001 s~%uint8       datum~%#uint8       reserved~%uint16      sb_list~%~%# INSNavGeodPosStdDev sub-block definition:~%# If the Position StdDev field is 1 then this sub block is available:~%float32     latitude_std_dev  # m~%float32     longitude_std_dev # m~%float32     height_std_dev    # m~%~%# INSNavGeodPosCov sub-block definition:~%# If the Position Cov field is 1 then this sub block is available:~%float32     latitude_longitude_cov # m^2~%float32     latitude_height_cov    # m^2~%float32     longitude_height_cov   # m^2~%~%# INSNavGeodAtt sub-block definition:~%# If the Attitude field is 1 then this sub block is available:~%float32     heading # deg~%float32     pitch   # deg~%float32     roll    # deg~%~%# INSNavGeodAttStdDev sub-block definition:~%# If the Attitude StdDev field is 1 then this sub block is available:~%float32     heading_std_dev # deg~%float32     pitch_std_dev   # deg~%float32     roll_std_dev    # deg~%~%# INSNavGeodAttCov sub-block definition:~%# If the Attitude Cov field is 1 then this sub block is available:~%float32     heading_pitch_cov # deg^2~%float32     heading_roll_cov  # deg^2~%float32     pitch_roll_cov    # deg^2~%~%# INSNavGeodVel sub-block definition:~%# If the Velocity field is 1 then this sub block is available:~%float32     ve # m/s~%float32     vn # m/s ~%float32     vu # m/s~%~%# INSNavGeodVelStdDev sub-block definition:~%# If the Velocity StdDev field is 1 then this sub block is available:~%float32     ve_std_dev # m/s~%float32     vn_std_dev # m/s~%float32     vu_std_dev # m/s~%~%# INSNavGeodVelCov sub-block definition:~%# If the Velocity Cov field is 1 then this sub block is available:~%float32     ve_vn_cov # m^2/s^2~%float32     ve_vu_cov # m^2/s^2~%float32     vn_vu_cov # m^2/s^2   ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <INSNavGeod>))
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
     4
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <INSNavGeod>))
  "Converts a ROS message object to a list"
  (cl:list 'INSNavGeod
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':gnss_mode (gnss_mode msg))
    (cl:cons ':error (error msg))
    (cl:cons ':info (info msg))
    (cl:cons ':gnss_age (gnss_age msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':height (height msg))
    (cl:cons ':undulation (undulation msg))
    (cl:cons ':accuracy (accuracy msg))
    (cl:cons ':latency (latency msg))
    (cl:cons ':datum (datum msg))
    (cl:cons ':sb_list (sb_list msg))
    (cl:cons ':latitude_std_dev (latitude_std_dev msg))
    (cl:cons ':longitude_std_dev (longitude_std_dev msg))
    (cl:cons ':height_std_dev (height_std_dev msg))
    (cl:cons ':latitude_longitude_cov (latitude_longitude_cov msg))
    (cl:cons ':latitude_height_cov (latitude_height_cov msg))
    (cl:cons ':longitude_height_cov (longitude_height_cov msg))
    (cl:cons ':heading (heading msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':heading_std_dev (heading_std_dev msg))
    (cl:cons ':pitch_std_dev (pitch_std_dev msg))
    (cl:cons ':roll_std_dev (roll_std_dev msg))
    (cl:cons ':heading_pitch_cov (heading_pitch_cov msg))
    (cl:cons ':heading_roll_cov (heading_roll_cov msg))
    (cl:cons ':pitch_roll_cov (pitch_roll_cov msg))
    (cl:cons ':ve (ve msg))
    (cl:cons ':vn (vn msg))
    (cl:cons ':vu (vu msg))
    (cl:cons ':ve_std_dev (ve_std_dev msg))
    (cl:cons ':vn_std_dev (vn_std_dev msg))
    (cl:cons ':vu_std_dev (vu_std_dev msg))
    (cl:cons ':ve_vn_cov (ve_vn_cov msg))
    (cl:cons ':ve_vu_cov (ve_vu_cov msg))
    (cl:cons ':vn_vu_cov (vn_vu_cov msg))
))
