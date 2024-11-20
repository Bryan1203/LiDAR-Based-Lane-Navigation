; Auto-generated. Do not edit!


(cl:in-package septentrio_gnss_driver-msg)


;//! \htmlinclude BlockHeader.msg.html

(cl:defclass <BlockHeader> (roslisp-msg-protocol:ros-message)
  ((sync_1
    :reader sync_1
    :initarg :sync_1
    :type cl:fixnum
    :initform 0)
   (sync_2
    :reader sync_2
    :initarg :sync_2
    :type cl:fixnum
    :initform 0)
   (crc
    :reader crc
    :initarg :crc
    :type cl:fixnum
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (revision
    :reader revision
    :initarg :revision
    :type cl:fixnum
    :initform 0)
   (length
    :reader length
    :initarg :length
    :type cl:fixnum
    :initform 0)
   (tow
    :reader tow
    :initarg :tow
    :type cl:integer
    :initform 0)
   (wnc
    :reader wnc
    :initarg :wnc
    :type cl:fixnum
    :initform 0))
)

(cl:defclass BlockHeader (<BlockHeader>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BlockHeader>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BlockHeader)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name septentrio_gnss_driver-msg:<BlockHeader> is deprecated: use septentrio_gnss_driver-msg:BlockHeader instead.")))

(cl:ensure-generic-function 'sync_1-val :lambda-list '(m))
(cl:defmethod sync_1-val ((m <BlockHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:sync_1-val is deprecated.  Use septentrio_gnss_driver-msg:sync_1 instead.")
  (sync_1 m))

(cl:ensure-generic-function 'sync_2-val :lambda-list '(m))
(cl:defmethod sync_2-val ((m <BlockHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:sync_2-val is deprecated.  Use septentrio_gnss_driver-msg:sync_2 instead.")
  (sync_2 m))

(cl:ensure-generic-function 'crc-val :lambda-list '(m))
(cl:defmethod crc-val ((m <BlockHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:crc-val is deprecated.  Use septentrio_gnss_driver-msg:crc instead.")
  (crc m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <BlockHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:id-val is deprecated.  Use septentrio_gnss_driver-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'revision-val :lambda-list '(m))
(cl:defmethod revision-val ((m <BlockHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:revision-val is deprecated.  Use septentrio_gnss_driver-msg:revision instead.")
  (revision m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <BlockHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:length-val is deprecated.  Use septentrio_gnss_driver-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'tow-val :lambda-list '(m))
(cl:defmethod tow-val ((m <BlockHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:tow-val is deprecated.  Use septentrio_gnss_driver-msg:tow instead.")
  (tow m))

(cl:ensure-generic-function 'wnc-val :lambda-list '(m))
(cl:defmethod wnc-val ((m <BlockHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader septentrio_gnss_driver-msg:wnc-val is deprecated.  Use septentrio_gnss_driver-msg:wnc instead.")
  (wnc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BlockHeader>) ostream)
  "Serializes a message object of type '<BlockHeader>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sync_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sync_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'crc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'crc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'revision)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'length)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tow)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tow)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tow)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tow)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wnc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'wnc)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BlockHeader>) istream)
  "Deserializes a message object of type '<BlockHeader>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sync_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sync_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'crc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'crc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'revision)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'length)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tow)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tow)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tow)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tow)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wnc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'wnc)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BlockHeader>)))
  "Returns string type for a message object of type '<BlockHeader>"
  "septentrio_gnss_driver/BlockHeader")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BlockHeader)))
  "Returns string type for a message object of type 'BlockHeader"
  "septentrio_gnss_driver/BlockHeader")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BlockHeader>)))
  "Returns md5sum for a message object of type '<BlockHeader>"
  "518ce7a1f9cbde747cc0c5b017e06f70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BlockHeader)))
  "Returns md5sum for a message object of type 'BlockHeader"
  "518ce7a1f9cbde747cc0c5b017e06f70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BlockHeader>)))
  "Returns full string definition for message of type '<BlockHeader>"
  (cl:format cl:nil "# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BlockHeader)))
  "Returns full string definition for message of type 'BlockHeader"
  (cl:format cl:nil "# Blockheader including time header for all ROS messages that publish SBF blocks~%~%uint8     sync_1~%uint8     sync_2~%uint16    crc~%uint16    id~%uint8     revision~%uint16    length~%~%uint32    tow # ms since week start~%uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BlockHeader>))
  (cl:+ 0
     1
     1
     2
     2
     1
     2
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BlockHeader>))
  "Converts a ROS message object to a list"
  (cl:list 'BlockHeader
    (cl:cons ':sync_1 (sync_1 msg))
    (cl:cons ':sync_2 (sync_2 msg))
    (cl:cons ':crc (crc msg))
    (cl:cons ':id (id msg))
    (cl:cons ':revision (revision msg))
    (cl:cons ':length (length msg))
    (cl:cons ':tow (tow msg))
    (cl:cons ':wnc (wnc msg))
))
