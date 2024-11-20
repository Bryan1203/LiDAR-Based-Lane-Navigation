;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::BlockHeader)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'BlockHeader (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::BLOCKHEADER")
  (make-package "SEPTENTRIO_GNSS_DRIVER::BLOCKHEADER"))

(in-package "ROS")
;;//! \htmlinclude BlockHeader.msg.html


(defclass septentrio_gnss_driver::BlockHeader
  :super ros::object
  :slots (_sync_1 _sync_2 _crc _id _revision _length _tow _wnc ))

(defmethod septentrio_gnss_driver::BlockHeader
  (:init
   (&key
    ((:sync_1 __sync_1) 0)
    ((:sync_2 __sync_2) 0)
    ((:crc __crc) 0)
    ((:id __id) 0)
    ((:revision __revision) 0)
    ((:length __length) 0)
    ((:tow __tow) 0)
    ((:wnc __wnc) 0)
    )
   (send-super :init)
   (setq _sync_1 (round __sync_1))
   (setq _sync_2 (round __sync_2))
   (setq _crc (round __crc))
   (setq _id (round __id))
   (setq _revision (round __revision))
   (setq _length (round __length))
   (setq _tow (round __tow))
   (setq _wnc (round __wnc))
   self)
  (:sync_1
   (&optional __sync_1)
   (if __sync_1 (setq _sync_1 __sync_1)) _sync_1)
  (:sync_2
   (&optional __sync_2)
   (if __sync_2 (setq _sync_2 __sync_2)) _sync_2)
  (:crc
   (&optional __crc)
   (if __crc (setq _crc __crc)) _crc)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:revision
   (&optional __revision)
   (if __revision (setq _revision __revision)) _revision)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:tow
   (&optional __tow)
   (if __tow (setq _tow __tow)) _tow)
  (:wnc
   (&optional __wnc)
   (if __wnc (setq _wnc __wnc)) _wnc)
  (:serialization-length
   ()
   (+
    ;; uint8 _sync_1
    1
    ;; uint8 _sync_2
    1
    ;; uint16 _crc
    2
    ;; uint16 _id
    2
    ;; uint8 _revision
    1
    ;; uint16 _length
    2
    ;; uint32 _tow
    4
    ;; uint16 _wnc
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _sync_1
       (write-byte _sync_1 s)
     ;; uint8 _sync_2
       (write-byte _sync_2 s)
     ;; uint16 _crc
       (write-word _crc s)
     ;; uint16 _id
       (write-word _id s)
     ;; uint8 _revision
       (write-byte _revision s)
     ;; uint16 _length
       (write-word _length s)
     ;; uint32 _tow
       (write-long _tow s)
     ;; uint16 _wnc
       (write-word _wnc s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _sync_1
     (setq _sync_1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sync_2
     (setq _sync_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _crc
     (setq _crc (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _revision
     (setq _revision (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _length
     (setq _length (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _tow
     (setq _tow (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _wnc
     (setq _wnc (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::BlockHeader :md5sum-) "518ce7a1f9cbde747cc0c5b017e06f70")
(setf (get septentrio_gnss_driver::BlockHeader :datatype-) "septentrio_gnss_driver/BlockHeader")
(setf (get septentrio_gnss_driver::BlockHeader :definition-)
      "# Blockheader including time header for all ROS messages that publish SBF blocks

uint8     sync_1
uint8     sync_2
uint16    crc
uint16    id
uint8     revision
uint16    length

uint32    tow # ms since week start
uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     
")



(provide :septentrio_gnss_driver/BlockHeader "518ce7a1f9cbde747cc0c5b017e06f70")


