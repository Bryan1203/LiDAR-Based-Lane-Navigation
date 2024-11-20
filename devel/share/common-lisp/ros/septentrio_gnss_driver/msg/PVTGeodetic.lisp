; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude PVTGeodetic.msg.html

(cl:defclass <PVTGeodetic> (roslisp-msg-protocol:ros-message)
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
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (error
    :reader error
    :initarg :error
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
   (vn
    :reader vn
    :initarg :vn
    :type cl:float
    :initform 0.0)
   (ve
    :reader ve
    :initarg :ve
    :type cl:float
    :initform 0.0)
   (vu
    :reader vu
    :initarg :vu
    :type cl:float
    :initform 0.0)
   (cog
    :reader cog
    :initarg :cog
    :type cl:float
    :initform 0.0)
   (rx_clk_bias
    :reader rx_clk_bias
    :initarg :rx_clk_bias
    :type cl:float
    :initform 0.0)
   (rx_clk_drift
    :reader rx_clk_drift
    :initarg :rx_clk_drift
    :type cl:float
    :initform 0.0)
   (time_system
    :reader time_system
    :initarg :time_system
    :type cl:fixnum
    :initform 0)
   (datum
    :reader datum
    :initarg :datum
    :type cl:fixnum
    :initform 0)
   (nr_sv
    :reader nr_sv
    :initarg :nr_sv
    :type cl:fixnum
    :initform 0)
   (wa_corr_info
    :reader wa_corr_info
    :initarg :wa_corr_info
    :type cl:fixnum
    :initform 0)
   (reference_id
    :reader reference_id
    :initarg :reference_id
    :type cl:fixnum
    :initform 0)
   (mean_corr_age
    :reader mean_corr_age
    :initarg :mean_corr_age
    :type cl:fixnum
    :initform 0)
   (signal_info
    :reader signal_info
    :initarg :signal_info
    :type cl:integer
    :initform 0)
   (alert_flag
    :reader alert_flag
    :initarg :alert_flag
    :type cl:fixnum
    :initform 0)
   (nr_bases
    :reader nr_bases
    :initarg :nr_bases
    :type cl:fixnum
    :initform 0)
   (ppp_info
    :reader ppp_info
    :initarg :ppp_info
    :type cl:fixnum
    :initform 0)
   (latency
    :reader latency
    :initarg :latency
    :type cl:fixnum
    :initform 0)
   (h_accuracy
    :reader h_accuracy
    :initarg :h_accuracy
    :type cl:fixnum
    :initform 0)
   (v_accuracy
    :reader v_accuracy
    :initarg :v_accuracy
    :type cl:fixnum
    :initform 0)
   (misc
    :reader misc
    :initarg :misc
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PVTGeodetic (<PVTGeodetic>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PVTGeodetic>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PVTGeodetic)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<PVTGeodetic> is deprecated: use septentrio_gnss_driver-msg:PVTGeodetic instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:header-val is deprecated.  Use septentrio_gnss_driver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_header-val :lambda-list '(m))
(cl:defmethod block_header-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:block_header-val is deprecated.  Use septentrio_gnss_driver-msg:block_header instead.")
  (block_header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:mode-val is deprecated.  Use septentrio_gnss_driver-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:error-val is deprecated.  Use septentrio_gnss_driver-msg:error instead.")
  (error m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:latitude-val is deprecated.  Use septentrio_gnss_driver-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:longitude-val is deprecated.  Use septentrio_gnss_driver-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:height-val is deprecated.  Use septentrio_gnss_driver-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'undulation-val :lambda-list '(m))
(cl:defmethod undulation-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:undulation-val is deprecated.  Use septentrio_gnss_driver-msg:undulation instead.")
  (undulation m))

(cl:ensure-generic-function 'vn-val :lambda-list '(m))
(cl:defmethod vn-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vn-val is deprecated.  Use septentrio_gnss_driver-msg:vn instead.")
  (vn m))

(cl:ensure-generic-function 've-val :lambda-list '(m))
(cl:defmethod ve-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:ve-val is deprecated.  Use septentrio_gnss_driver-msg:ve instead.")
  (ve m))

(cl:ensure-generic-function 'vu-val :lambda-list '(m))
(cl:defmethod vu-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:vu-val is deprecated.  Use septentrio_gnss_driver-msg:vu instead.")
  (vu m))

(cl:ensure-generic-function 'cog-val :lambda-list '(m))
(cl:defmethod cog-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:cog-val is deprecated.  Use septentrio_gnss_driver-msg:cog instead.")
  (cog m))

(cl:ensure-generic-function 'rx_clk_bias-val :lambda-list '(m))
(cl:defmethod rx_clk_bias-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:rx_clk_bias-val is deprecated.  Use septentrio_gnss_driver-msg:rx_clk_bias instead.")
  (rx_clk_bias m))

(cl:ensure-generic-function 'rx_clk_drift-val :lambda-list '(m))
(cl:defmethod rx_clk_drift-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:rx_clk_drift-val is deprecated.  Use septentrio_gnss_driver-msg:rx_clk_drift instead.")
  (rx_clk_drift m))

(cl:ensure-generic-function 'time_system-val :lambda-list '(m))
(cl:defmethod time_system-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:time_system-val is deprecated.  Use septentrio_gnss_driver-msg:time_system instead.")
  (time_system m))

(cl:ensure-generic-function 'datum-val :lambda-list '(m))
(cl:defmethod datum-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:datum-val is deprecated.  Use septentrio_gnss_driver-msg:datum instead.")
  (datum m))

(cl:ensure-generic-function 'nr_sv-val :lambda-list '(m))
(cl:defmethod nr_sv-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:nr_sv-val is deprecated.  Use septentrio_gnss_driver-msg:nr_sv instead.")
  (nr_sv m))

(cl:ensure-generic-function 'wa_corr_info-val :lambda-list '(m))
(cl:defmethod wa_corr_info-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:wa_corr_info-val is deprecated.  Use septentrio_gnss_driver-msg:wa_corr_info instead.")
  (wa_corr_info m))

(cl:ensure-generic-function 'reference_id-val :lambda-list '(m))
(cl:defmethod reference_id-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:reference_id-val is deprecated.  Use septentrio_gnss_driver-msg:reference_id instead.")
  (reference_id m))

(cl:ensure-generic-function 'mean_corr_age-val :lambda-list '(m))
(cl:defmethod mean_corr_age-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:mean_corr_age-val is deprecated.  Use septentrio_gnss_driver-msg:mean_corr_age instead.")
  (mean_corr_age m))

(cl:ensure-generic-function 'signal_info-val :lambda-list '(m))
(cl:defmethod signal_info-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:signal_info-val is deprecated.  Use septentrio_gnss_driver-msg:signal_info instead.")
  (signal_info m))

(cl:ensure-generic-function 'alert_flag-val :lambda-list '(m))
(cl:defmethod alert_flag-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:alert_flag-val is deprecated.  Use septentrio_gnss_driver-msg:alert_flag instead.")
  (alert_flag m))

(cl:ensure-generic-function 'nr_bases-val :lambda-list '(m))
(cl:defmethod nr_bases-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:nr_bases-val is deprecated.  Use septentrio_gnss_driver-msg:nr_bases instead.")
  (nr_bases m))

(cl:ensure-generic-function 'ppp_info-val :lambda-list '(m))
(cl:defmethod ppp_info-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:ppp_info-val is deprecated.  Use septentrio_gnss_driver-msg:ppp_info instead.")
  (ppp_info m))

(cl:ensure-generic-function 'latency-val :lambda-list '(m))
(cl:defmethod latency-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:latency-val is deprecated.  Use septentrio_gnss_driver-msg:latency instead.")
  (latency m))

(cl:ensure-generic-function 'h_accuracy-val :lambda-list '(m))
(cl:defmethod h_accuracy-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:h_accuracy-val is deprecated.  Use septentrio_gnss_driver-msg:h_accuracy instead.")
  (h_accuracy m))

(cl:ensure-generic-function 'v_accuracy-val :lambda-list '(m))
(cl:defmethod v_accuracy-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:v_accuracy-val is deprecated.  Use septentrio_gnss_driver-msg:v_accuracy instead.")
  (v_accuracy m))

(cl:ensure-generic-function 'misc-val :lambda-list '(m))
(cl:defmethod misc-val ((m <PVTGeodetic>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:misc-val is deprecated.  Use septentrio_gnss_driver-msg:misc instead.")
  (misc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PVTGeodetic>) ostream)
  "Serializes a message object of type '<PVTGeodetic>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'block_header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 've))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vu))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cog))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rx_clk_bias))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rx_clk_drift))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_system)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'datum)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nr_sv)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wa_corr_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mean_corr_age)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mean_corr_age)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'signal_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'signal_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'signal_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'signal_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'alert_flag)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nr_bases)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ppp_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ppp_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'latency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'latency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'h_accuracy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'h_accuracy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'v_accuracy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'v_accuracy)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'misc)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PVTGeodetic>) istream)
  "Deserializes a message object of type '<PVTGeodetic>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'block_header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
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
    (cl:setf (cl:slot-value msg 've) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'cog) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rx_clk_bias) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rx_clk_drift) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_system)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'datum)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nr_sv)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wa_corr_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reference_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reference_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mean_corr_age)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mean_corr_age)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'signal_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'signal_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'signal_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'signal_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'alert_flag)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nr_bases)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ppp_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ppp_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'latency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'latency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'h_accuracy)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'h_accuracy)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'v_accuracy)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'v_accuracy)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'misc)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PVTGeodetic>)))
  "Returns string type for a message object of type '<PVTGeodetic>"
  "septentrio_gnss_driver/PVTGeodetic")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PVTGeodetic)))
  "Returns string type for a message object of type 'PVTGeodetic"
  "septentrio_gnss_driver/PVTGeodetic")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PVTGeodetic>)))
  "Returns md5sum for a message object of type '<PVTGeodetic>"
  "4d3911a0efd862e13c076ba7d322cd3e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PVTGeodetic)))
  "Returns md5sum for a message object of type 'PVTGeodetic"
  "4d3911a0efd862e13c076ba7d322cd3e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PVTGeodetic>)))
  "Returns full string definition for message of type '<PVTGeodetic>"
  (cl:format cl:nil "# PVTGeodetic block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        mode~%uint8        error        ~%float64      latitude      # rad    ~%float64      longitude     # rad~%float64      height        # m (ellipsoidal)~%float32      undulation    # m~%float32      vn            # m/s~%float32      ve            # m/s~%float32      vu            # m/s~%float32      cog           # deg~%float64      rx_clk_bias   # ms~%float32      rx_clk_drift  # ppm~%uint8        time_system ~%uint8        datum~%uint8        nr_sv ~%uint8        wa_corr_info~%uint16       reference_id~%uint16       mean_corr_age # 0.01s~%uint32       signal_info~%uint8        alert_flag~%uint8        nr_bases~%uint16       ppp_info      # s~%uint16       latency       # 0.0001 s~%uint16       h_accuracy    # 0.01 m~%uint16       v_accuracy    # 0.01 m~%uint8        misc~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PVTGeodetic)))
  "Returns full string definition for message of type 'PVTGeodetic"
  (cl:format cl:nil "# PVTGeodetic block~%# ROS message header~%std_msgs/Header header~%~%# SBF block header including time header~%BlockHeader  block_header   ~%~%uint8        mode~%uint8        error        ~%float64      latitude      # rad    ~%float64      longitude     # rad~%float64      height        # m (ellipsoidal)~%float32      undulation    # m~%float32      vn            # m/s~%float32      ve            # m/s~%float32      vu            # m/s~%float32      cog           # deg~%float64      rx_clk_bias   # ms~%float32      rx_clk_drift  # ppm~%uint8        time_system ~%uint8        datum~%uint8        nr_sv ~%uint8        wa_corr_info~%uint16       reference_id~%uint16       mean_corr_age # 0.01s~%uint32       signal_info~%uint8        alert_flag~%uint8        nr_bases~%uint16       ppp_info      # s~%uint16       latency       # 0.0001 s~%uint16       h_accuracy    # 0.01 m~%uint16       v_accuracy    # 0.01 m~%uint8        misc~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: septentrio_gnss_driver/BlockHeader~%# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PVTGeodetic>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'block_header))
     1
     1
     8
     8
     8
     4
     4
     4
     4
     4
     8
     4
     1
     1
     1
     1
     2
     2
     4
     1
     1
     2
     2
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PVTGeodetic>))
  "Converts a ROS message object to a list"
  (cl:list 'PVTGeodetic
    (cl:cons ':header (header msg))
    (cl:cons ':block_header (block_header msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':error (error msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':height (height msg))
    (cl:cons ':undulation (undulation msg))
    (cl:cons ':vn (vn msg))
    (cl:cons ':ve (ve msg))
    (cl:cons ':vu (vu msg))
    (cl:cons ':cog (cog msg))
    (cl:cons ':rx_clk_bias (rx_clk_bias msg))
    (cl:cons ':rx_clk_drift (rx_clk_drift msg))
    (cl:cons ':time_system (time_system msg))
    (cl:cons ':datum (datum msg))
    (cl:cons ':nr_sv (nr_sv msg))
    (cl:cons ':wa_corr_info (wa_corr_info msg))
    (cl:cons ':reference_id (reference_id msg))
    (cl:cons ':mean_corr_age (mean_corr_age msg))
    (cl:cons ':signal_info (signal_info msg))
    (cl:cons ':alert_flag (alert_flag msg))
    (cl:cons ':nr_bases (nr_bases msg))
    (cl:cons ':ppp_info (ppp_info msg))
    (cl:cons ':latency (latency msg))
    (cl:cons ':h_accuracy (h_accuracy msg))
    (cl:cons ':v_accuracy (v_accuracy msg))
    (cl:cons ':misc (misc msg))
))
