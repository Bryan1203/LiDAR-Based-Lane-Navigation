;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::MeasEpochChannelType2)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'MeasEpochChannelType2 (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::MEASEPOCHCHANNELTYPE2")
  (make-package "SEPTENTRIO_GNSS_DRIVER::MEASEPOCHCHANNELTYPE2"))

(in-package "ROS")
;;//! \htmlinclude MeasEpochChannelType2.msg.html


(defclass septentrio_gnss_driver::MeasEpochChannelType2
  :super ros::object
  :slots (_type _lock_time _cn0 _offsets_msb _carrier_msb _obs_info _code_offset_lsb _carrier_lsb _doppler_offset_lsb ))

(defmethod septentrio_gnss_driver::MeasEpochChannelType2
  (:init
   (&key
    ((:type __type) 0)
    ((:lock_time __lock_time) 0)
    ((:cn0 __cn0) 0)
    ((:offsets_msb __offsets_msb) 0)
    ((:carrier_msb __carrier_msb) 0)
    ((:obs_info __obs_info) 0)
    ((:code_offset_lsb __code_offset_lsb) 0)
    ((:carrier_lsb __carrier_lsb) 0)
    ((:doppler_offset_lsb __doppler_offset_lsb) 0)
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _lock_time (round __lock_time))
   (setq _cn0 (round __cn0))
   (setq _offsets_msb (round __offsets_msb))
   (setq _carrier_msb (round __carrier_msb))
   (setq _obs_info (round __obs_info))
   (setq _code_offset_lsb (round __code_offset_lsb))
   (setq _carrier_lsb (round __carrier_lsb))
   (setq _doppler_offset_lsb (round __doppler_offset_lsb))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:lock_time
   (&optional __lock_time)
   (if __lock_time (setq _lock_time __lock_time)) _lock_time)
  (:cn0
   (&optional __cn0)
   (if __cn0 (setq _cn0 __cn0)) _cn0)
  (:offsets_msb
   (&optional __offsets_msb)
   (if __offsets_msb (setq _offsets_msb __offsets_msb)) _offsets_msb)
  (:carrier_msb
   (&optional __carrier_msb)
   (if __carrier_msb (setq _carrier_msb __carrier_msb)) _carrier_msb)
  (:obs_info
   (&optional __obs_info)
   (if __obs_info (setq _obs_info __obs_info)) _obs_info)
  (:code_offset_lsb
   (&optional __code_offset_lsb)
   (if __code_offset_lsb (setq _code_offset_lsb __code_offset_lsb)) _code_offset_lsb)
  (:carrier_lsb
   (&optional __carrier_lsb)
   (if __carrier_lsb (setq _carrier_lsb __carrier_lsb)) _carrier_lsb)
  (:doppler_offset_lsb
   (&optional __doppler_offset_lsb)
   (if __doppler_offset_lsb (setq _doppler_offset_lsb __doppler_offset_lsb)) _doppler_offset_lsb)
  (:serialization-length
   ()
   (+
    ;; uint8 _type
    1
    ;; uint8 _lock_time
    1
    ;; uint8 _cn0
    1
    ;; uint8 _offsets_msb
    1
    ;; int8 _carrier_msb
    1
    ;; uint8 _obs_info
    1
    ;; uint16 _code_offset_lsb
    2
    ;; uint16 _carrier_lsb
    2
    ;; uint16 _doppler_offset_lsb
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _type
       (write-byte _type s)
     ;; uint8 _lock_time
       (write-byte _lock_time s)
     ;; uint8 _cn0
       (write-byte _cn0 s)
     ;; uint8 _offsets_msb
       (write-byte _offsets_msb s)
     ;; int8 _carrier_msb
       (write-byte _carrier_msb s)
     ;; uint8 _obs_info
       (write-byte _obs_info s)
     ;; uint16 _code_offset_lsb
       (write-word _code_offset_lsb s)
     ;; uint16 _carrier_lsb
       (write-word _carrier_lsb s)
     ;; uint16 _doppler_offset_lsb
       (write-word _doppler_offset_lsb s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _lock_time
     (setq _lock_time (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cn0
     (setq _cn0 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _offsets_msb
     (setq _offsets_msb (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int8 _carrier_msb
     (setq _carrier_msb (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _carrier_msb 127) (setq _carrier_msb (- _carrier_msb 256)))
   ;; uint8 _obs_info
     (setq _obs_info (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _code_offset_lsb
     (setq _code_offset_lsb (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _carrier_lsb
     (setq _carrier_lsb (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _doppler_offset_lsb
     (setq _doppler_offset_lsb (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::MeasEpochChannelType2 :md5sum-) "204efbe40d2e56cd210b50c219f3a400")
(setf (get septentrio_gnss_driver::MeasEpochChannelType2 :datatype-) "septentrio_gnss_driver/MeasEpochChannelType2")
(setf (get septentrio_gnss_driver::MeasEpochChannelType2 :definition-)
      "# MeasEpochChannelType2 block
   
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



(provide :septentrio_gnss_driver/MeasEpochChannelType2 "204efbe40d2e56cd210b50c219f3a400")


