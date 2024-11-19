;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::MeasEpochChannelType1)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'MeasEpochChannelType1 (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::MEASEPOCHCHANNELTYPE1")
  (make-package "SEPTENTRIO_GNSS_DRIVER::MEASEPOCHCHANNELTYPE1"))

(in-package "ROS")
;;//! \htmlinclude MeasEpochChannelType1.msg.html


(defclass septentrio_gnss_driver::MeasEpochChannelType1
  :super ros::object
  :slots (_rx_channel _type _sv_id _misc _code_lsb _doppler _carrier_lsb _carrier_msb _cn0 _lock_time _obs_info _n2 _type2 ))

(defmethod septentrio_gnss_driver::MeasEpochChannelType1
  (:init
   (&key
    ((:rx_channel __rx_channel) 0)
    ((:type __type) 0)
    ((:sv_id __sv_id) 0)
    ((:misc __misc) 0)
    ((:code_lsb __code_lsb) 0)
    ((:doppler __doppler) 0)
    ((:carrier_lsb __carrier_lsb) 0)
    ((:carrier_msb __carrier_msb) 0)
    ((:cn0 __cn0) 0)
    ((:lock_time __lock_time) 0)
    ((:obs_info __obs_info) 0)
    ((:n2 __n2) 0)
    ((:type2 __type2) ())
    )
   (send-super :init)
   (setq _rx_channel (round __rx_channel))
   (setq _type (round __type))
   (setq _sv_id (round __sv_id))
   (setq _misc (round __misc))
   (setq _code_lsb (round __code_lsb))
   (setq _doppler (round __doppler))
   (setq _carrier_lsb (round __carrier_lsb))
   (setq _carrier_msb (round __carrier_msb))
   (setq _cn0 (round __cn0))
   (setq _lock_time (round __lock_time))
   (setq _obs_info (round __obs_info))
   (setq _n2 (round __n2))
   (setq _type2 __type2)
   self)
  (:rx_channel
   (&optional __rx_channel)
   (if __rx_channel (setq _rx_channel __rx_channel)) _rx_channel)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:sv_id
   (&optional __sv_id)
   (if __sv_id (setq _sv_id __sv_id)) _sv_id)
  (:misc
   (&optional __misc)
   (if __misc (setq _misc __misc)) _misc)
  (:code_lsb
   (&optional __code_lsb)
   (if __code_lsb (setq _code_lsb __code_lsb)) _code_lsb)
  (:doppler
   (&optional __doppler)
   (if __doppler (setq _doppler __doppler)) _doppler)
  (:carrier_lsb
   (&optional __carrier_lsb)
   (if __carrier_lsb (setq _carrier_lsb __carrier_lsb)) _carrier_lsb)
  (:carrier_msb
   (&optional __carrier_msb)
   (if __carrier_msb (setq _carrier_msb __carrier_msb)) _carrier_msb)
  (:cn0
   (&optional __cn0)
   (if __cn0 (setq _cn0 __cn0)) _cn0)
  (:lock_time
   (&optional __lock_time)
   (if __lock_time (setq _lock_time __lock_time)) _lock_time)
  (:obs_info
   (&optional __obs_info)
   (if __obs_info (setq _obs_info __obs_info)) _obs_info)
  (:n2
   (&optional __n2)
   (if __n2 (setq _n2 __n2)) _n2)
  (:type2
   (&rest __type2)
   (if (keywordp (car __type2))
       (send* _type2 __type2)
     (progn
       (if __type2 (setq _type2 (car __type2)))
       _type2)))
  (:serialization-length
   ()
   (+
    ;; uint8 _rx_channel
    1
    ;; uint8 _type
    1
    ;; uint8 _sv_id
    1
    ;; uint8 _misc
    1
    ;; uint32 _code_lsb
    4
    ;; int32 _doppler
    4
    ;; uint16 _carrier_lsb
    2
    ;; int8 _carrier_msb
    1
    ;; uint8 _cn0
    1
    ;; uint16 _lock_time
    2
    ;; uint8 _obs_info
    1
    ;; uint8 _n2
    1
    ;; septentrio_gnss_driver/MeasEpochChannelType2[] _type2
    (apply #'+ (send-all _type2 :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _rx_channel
       (write-byte _rx_channel s)
     ;; uint8 _type
       (write-byte _type s)
     ;; uint8 _sv_id
       (write-byte _sv_id s)
     ;; uint8 _misc
       (write-byte _misc s)
     ;; uint32 _code_lsb
       (write-long _code_lsb s)
     ;; int32 _doppler
       (write-long _doppler s)
     ;; uint16 _carrier_lsb
       (write-word _carrier_lsb s)
     ;; int8 _carrier_msb
       (write-byte _carrier_msb s)
     ;; uint8 _cn0
       (write-byte _cn0 s)
     ;; uint16 _lock_time
       (write-word _lock_time s)
     ;; uint8 _obs_info
       (write-byte _obs_info s)
     ;; uint8 _n2
       (write-byte _n2 s)
     ;; septentrio_gnss_driver/MeasEpochChannelType2[] _type2
     (write-long (length _type2) s)
     (dolist (elem _type2)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _rx_channel
     (setq _rx_channel (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sv_id
     (setq _sv_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _misc
     (setq _misc (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _code_lsb
     (setq _code_lsb (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _doppler
     (setq _doppler (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _carrier_lsb
     (setq _carrier_lsb (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int8 _carrier_msb
     (setq _carrier_msb (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _carrier_msb 127) (setq _carrier_msb (- _carrier_msb 256)))
   ;; uint8 _cn0
     (setq _cn0 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _lock_time
     (setq _lock_time (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _obs_info
     (setq _obs_info (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _n2
     (setq _n2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; septentrio_gnss_driver/MeasEpochChannelType2[] _type2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _type2 (let (r) (dotimes (i n) (push (instance septentrio_gnss_driver::MeasEpochChannelType2 :init) r)) r))
     (dolist (elem- _type2)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::MeasEpochChannelType1 :md5sum-) "6ec5531728aded72fdbf7e7550cabd87")
(setf (get septentrio_gnss_driver::MeasEpochChannelType1 :datatype-) "septentrio_gnss_driver/MeasEpochChannelType1")
(setf (get septentrio_gnss_driver::MeasEpochChannelType1 :definition-)
      "# MeasEpochChannelType1 block
   
uint8  rx_channel
uint8  type
uint8  sv_id
uint8  misc        # 4294967.296 m
uint32 code_lsb    # 0.001 m
int32  doppler     # 0.0001 Hz
uint16 carrier_lsb # 0.001 cycles
int8   carrier_msb # 65.536 cycles
uint8  cn0         # 0.25 dB-Hz
uint16 lock_time
uint8  obs_info
uint8  n2

MeasEpochChannelType2[] type2
================================================================================
MSG: septentrio_gnss_driver/MeasEpochChannelType2
# MeasEpochChannelType2 block
   
uint8  type
uint8  lock_time          # s
uint8  cn0                # 0.25 dB-Hz
uint8  offsets_msb        # 65.536 m or 65.536 Hz
int8   carrier_msb        # 65.536 cycles
uint8  obs_info
uint16 code_offset_lsb    # 0.001 m
uint16 carrier_lsb        # 0.001 cycles
uint16 doppler_offset_lsb # 0.0001 Hz
")



(provide :septentrio_gnss_driver/MeasEpochChannelType1 "6ec5531728aded72fdbf7e7550cabd87")


