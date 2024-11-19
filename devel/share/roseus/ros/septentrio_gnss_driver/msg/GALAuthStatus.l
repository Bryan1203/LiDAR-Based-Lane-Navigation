;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::GALAuthStatus)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'GALAuthStatus (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::GALAUTHSTATUS")
  (make-package "SEPTENTRIO_GNSS_DRIVER::GALAUTHSTATUS"))

(in-package "ROS")
;;//! \htmlinclude GALAuthStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::GALAuthStatus
  :super ros::object
  :slots (_header _block_header _osnma_status _trusted_time_delta _gal_active_mask _gal_authentic_mask _gps_active_mask _gps_authentic_mask ))

(defmethod septentrio_gnss_driver::GALAuthStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:osnma_status __osnma_status) 0)
    ((:trusted_time_delta __trusted_time_delta) 0.0)
    ((:gal_active_mask __gal_active_mask) 0)
    ((:gal_authentic_mask __gal_authentic_mask) 0)
    ((:gps_active_mask __gps_active_mask) 0)
    ((:gps_authentic_mask __gps_authentic_mask) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _osnma_status (round __osnma_status))
   (setq _trusted_time_delta (float __trusted_time_delta))
   (setq _gal_active_mask (round __gal_active_mask))
   (setq _gal_authentic_mask (round __gal_authentic_mask))
   (setq _gps_active_mask (round __gps_active_mask))
   (setq _gps_authentic_mask (round __gps_authentic_mask))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:block_header
   (&rest __block_header)
   (if (keywordp (car __block_header))
       (send* _block_header __block_header)
     (progn
       (if __block_header (setq _block_header (car __block_header)))
       _block_header)))
  (:osnma_status
   (&optional __osnma_status)
   (if __osnma_status (setq _osnma_status __osnma_status)) _osnma_status)
  (:trusted_time_delta
   (&optional __trusted_time_delta)
   (if __trusted_time_delta (setq _trusted_time_delta __trusted_time_delta)) _trusted_time_delta)
  (:gal_active_mask
   (&optional __gal_active_mask)
   (if __gal_active_mask (setq _gal_active_mask __gal_active_mask)) _gal_active_mask)
  (:gal_authentic_mask
   (&optional __gal_authentic_mask)
   (if __gal_authentic_mask (setq _gal_authentic_mask __gal_authentic_mask)) _gal_authentic_mask)
  (:gps_active_mask
   (&optional __gps_active_mask)
   (if __gps_active_mask (setq _gps_active_mask __gps_active_mask)) _gps_active_mask)
  (:gps_authentic_mask
   (&optional __gps_authentic_mask)
   (if __gps_authentic_mask (setq _gps_authentic_mask __gps_authentic_mask)) _gps_authentic_mask)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; septentrio_gnss_driver/BlockHeader _block_header
    (send _block_header :serialization-length)
    ;; uint16 _osnma_status
    2
    ;; float32 _trusted_time_delta
    4
    ;; uint64 _gal_active_mask
    8
    ;; uint64 _gal_authentic_mask
    8
    ;; uint64 _gps_active_mask
    8
    ;; uint64 _gps_authentic_mask
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; septentrio_gnss_driver/BlockHeader _block_header
       (send _block_header :serialize s)
     ;; uint16 _osnma_status
       (write-word _osnma_status s)
     ;; float32 _trusted_time_delta
       (sys::poke _trusted_time_delta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint64 _gal_active_mask
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _gal_active_mask (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _gal_active_mask) (= (length (_gal_active_mask . bv)) 2)) ;; bignum
              (write-long (ash (elt (_gal_active_mask . bv) 0) 0) s)
              (write-long (ash (elt (_gal_active_mask . bv) 1) -1) s))
             ((and (class _gal_active_mask) (= (length (_gal_active_mask . bv)) 1)) ;; big1
              (write-long (elt (_gal_active_mask . bv) 0) s)
              (write-long (if (>= _gal_active_mask 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _gal_active_mask s)(write-long (if (>= _gal_active_mask 0) 0 #xffffffff) s)))
     ;; uint64 _gal_authentic_mask
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _gal_authentic_mask (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _gal_authentic_mask) (= (length (_gal_authentic_mask . bv)) 2)) ;; bignum
              (write-long (ash (elt (_gal_authentic_mask . bv) 0) 0) s)
              (write-long (ash (elt (_gal_authentic_mask . bv) 1) -1) s))
             ((and (class _gal_authentic_mask) (= (length (_gal_authentic_mask . bv)) 1)) ;; big1
              (write-long (elt (_gal_authentic_mask . bv) 0) s)
              (write-long (if (>= _gal_authentic_mask 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _gal_authentic_mask s)(write-long (if (>= _gal_authentic_mask 0) 0 #xffffffff) s)))
     ;; uint64 _gps_active_mask
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _gps_active_mask (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _gps_active_mask) (= (length (_gps_active_mask . bv)) 2)) ;; bignum
              (write-long (ash (elt (_gps_active_mask . bv) 0) 0) s)
              (write-long (ash (elt (_gps_active_mask . bv) 1) -1) s))
             ((and (class _gps_active_mask) (= (length (_gps_active_mask . bv)) 1)) ;; big1
              (write-long (elt (_gps_active_mask . bv) 0) s)
              (write-long (if (>= _gps_active_mask 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _gps_active_mask s)(write-long (if (>= _gps_active_mask 0) 0 #xffffffff) s)))
     ;; uint64 _gps_authentic_mask
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _gps_authentic_mask (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _gps_authentic_mask) (= (length (_gps_authentic_mask . bv)) 2)) ;; bignum
              (write-long (ash (elt (_gps_authentic_mask . bv) 0) 0) s)
              (write-long (ash (elt (_gps_authentic_mask . bv) 1) -1) s))
             ((and (class _gps_authentic_mask) (= (length (_gps_authentic_mask . bv)) 1)) ;; big1
              (write-long (elt (_gps_authentic_mask . bv) 0) s)
              (write-long (if (>= _gps_authentic_mask 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _gps_authentic_mask s)(write-long (if (>= _gps_authentic_mask 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; septentrio_gnss_driver/BlockHeader _block_header
     (send _block_header :deserialize buf ptr-) (incf ptr- (send _block_header :serialization-length))
   ;; uint16 _osnma_status
     (setq _osnma_status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _trusted_time_delta
     (setq _trusted_time_delta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint64 _gal_active_mask
     
#+(or :alpha :irix6 :x86_64)
      (setf _gal_active_mask (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _gal_active_mask (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _gal_authentic_mask
     
#+(or :alpha :irix6 :x86_64)
      (setf _gal_authentic_mask (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _gal_authentic_mask (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _gps_active_mask
     
#+(or :alpha :irix6 :x86_64)
      (setf _gps_active_mask (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _gps_active_mask (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _gps_authentic_mask
     
#+(or :alpha :irix6 :x86_64)
      (setf _gps_authentic_mask (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _gps_authentic_mask (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::GALAuthStatus :md5sum-) "9b9a0d56668644a236059d089716150f")
(setf (get septentrio_gnss_driver::GALAuthStatus :datatype-) "septentrio_gnss_driver/GALAuthStatus")
(setf (get septentrio_gnss_driver::GALAuthStatus :definition-)
      "# GALAuthStatus block 
# Block_Number 4245

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint16  osnma_status
float32 trusted_time_delta # s
uint64  gal_active_mask
uint64  gal_authentic_mask
uint64  gps_active_mask
uint64  gps_authentic_mask
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: septentrio_gnss_driver/BlockHeader
# Blockheader including time header for all ROS messages that publish SBF blocks

uint8     sync_1
uint8     sync_2
uint16    crc
uint16    id
uint8     revision
uint16    length

uint32    tow # ms since week start
uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     
")



(provide :septentrio_gnss_driver/GALAuthStatus "9b9a0d56668644a236059d089716150f")


