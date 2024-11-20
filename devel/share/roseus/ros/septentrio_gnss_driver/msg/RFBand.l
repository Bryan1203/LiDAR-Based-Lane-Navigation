;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::RFBand)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'RFBand (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::RFBAND")
  (make-package "SEPTENTRIO_GNSS_DRIVER::RFBAND"))

(in-package "ROS")
;;//! \htmlinclude RFBand.msg.html


(defclass septentrio_gnss_driver::RFBand
  :super ros::object
  :slots (_frequency _bandwidth _info ))

(defmethod septentrio_gnss_driver::RFBand
  (:init
   (&key
    ((:frequency __frequency) 0)
    ((:bandwidth __bandwidth) 0)
    ((:info __info) 0)
    )
   (send-super :init)
   (setq _frequency (round __frequency))
   (setq _bandwidth (round __bandwidth))
   (setq _info (round __info))
   self)
  (:frequency
   (&optional __frequency)
   (if __frequency (setq _frequency __frequency)) _frequency)
  (:bandwidth
   (&optional __bandwidth)
   (if __bandwidth (setq _bandwidth __bandwidth)) _bandwidth)
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:serialization-length
   ()
   (+
    ;; uint32 _frequency
    4
    ;; uint16 _bandwidth
    2
    ;; uint8 _info
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _frequency
       (write-long _frequency s)
     ;; uint16 _bandwidth
       (write-word _bandwidth s)
     ;; uint8 _info
       (write-byte _info s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _frequency
     (setq _frequency (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _bandwidth
     (setq _bandwidth (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _info
     (setq _info (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::RFBand :md5sum-) "987cd35fc563b11daae475e3e9a37fd6")
(setf (get septentrio_gnss_driver::RFBand :datatype-) "septentrio_gnss_driver/RFBand")
(setf (get septentrio_gnss_driver::RFBand :definition-)
      "# RFband block

uint32 frequency # Hz
uint16 bandwidth # kHz
uint8  info
")



(provide :septentrio_gnss_driver/RFBand "987cd35fc563b11daae475e3e9a37fd6")


