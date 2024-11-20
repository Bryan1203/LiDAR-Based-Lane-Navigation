;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::VectorInfoCart)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'VectorInfoCart (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::VECTORINFOCART")
  (make-package "SEPTENTRIO_GNSS_DRIVER::VECTORINFOCART"))

(in-package "ROS")
;;//! \htmlinclude VectorInfoCart.msg.html


(defclass septentrio_gnss_driver::VectorInfoCart
  :super ros::object
  :slots (_nr_sv _error _mode _misc _delta_x _delta_y _delta_z _delta_vx _delta_vy _delta_vz _azimuth _elevation _reference_id _corr_age _signal_info ))

(defmethod septentrio_gnss_driver::VectorInfoCart
  (:init
   (&key
    ((:nr_sv __nr_sv) 0)
    ((:error __error) 0)
    ((:mode __mode) 0)
    ((:misc __misc) 0)
    ((:delta_x __delta_x) 0.0)
    ((:delta_y __delta_y) 0.0)
    ((:delta_z __delta_z) 0.0)
    ((:delta_vx __delta_vx) 0.0)
    ((:delta_vy __delta_vy) 0.0)
    ((:delta_vz __delta_vz) 0.0)
    ((:azimuth __azimuth) 0)
    ((:elevation __elevation) 0)
    ((:reference_id __reference_id) 0)
    ((:corr_age __corr_age) 0)
    ((:signal_info __signal_info) 0)
    )
   (send-super :init)
   (setq _nr_sv (round __nr_sv))
   (setq _error (round __error))
   (setq _mode (round __mode))
   (setq _misc (round __misc))
   (setq _delta_x (float __delta_x))
   (setq _delta_y (float __delta_y))
   (setq _delta_z (float __delta_z))
   (setq _delta_vx (float __delta_vx))
   (setq _delta_vy (float __delta_vy))
   (setq _delta_vz (float __delta_vz))
   (setq _azimuth (round __azimuth))
   (setq _elevation (round __elevation))
   (setq _reference_id (round __reference_id))
   (setq _corr_age (round __corr_age))
   (setq _signal_info (round __signal_info))
   self)
  (:nr_sv
   (&optional __nr_sv)
   (if __nr_sv (setq _nr_sv __nr_sv)) _nr_sv)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:misc
   (&optional __misc)
   (if __misc (setq _misc __misc)) _misc)
  (:delta_x
   (&optional __delta_x)
   (if __delta_x (setq _delta_x __delta_x)) _delta_x)
  (:delta_y
   (&optional __delta_y)
   (if __delta_y (setq _delta_y __delta_y)) _delta_y)
  (:delta_z
   (&optional __delta_z)
   (if __delta_z (setq _delta_z __delta_z)) _delta_z)
  (:delta_vx
   (&optional __delta_vx)
   (if __delta_vx (setq _delta_vx __delta_vx)) _delta_vx)
  (:delta_vy
   (&optional __delta_vy)
   (if __delta_vy (setq _delta_vy __delta_vy)) _delta_vy)
  (:delta_vz
   (&optional __delta_vz)
   (if __delta_vz (setq _delta_vz __delta_vz)) _delta_vz)
  (:azimuth
   (&optional __azimuth)
   (if __azimuth (setq _azimuth __azimuth)) _azimuth)
  (:elevation
   (&optional __elevation)
   (if __elevation (setq _elevation __elevation)) _elevation)
  (:reference_id
   (&optional __reference_id)
   (if __reference_id (setq _reference_id __reference_id)) _reference_id)
  (:corr_age
   (&optional __corr_age)
   (if __corr_age (setq _corr_age __corr_age)) _corr_age)
  (:signal_info
   (&optional __signal_info)
   (if __signal_info (setq _signal_info __signal_info)) _signal_info)
  (:serialization-length
   ()
   (+
    ;; uint8 _nr_sv
    1
    ;; uint8 _error
    1
    ;; uint8 _mode
    1
    ;; uint8 _misc
    1
    ;; float64 _delta_x
    8
    ;; float64 _delta_y
    8
    ;; float64 _delta_z
    8
    ;; float32 _delta_vx
    4
    ;; float32 _delta_vy
    4
    ;; float32 _delta_vz
    4
    ;; uint16 _azimuth
    2
    ;; int16 _elevation
    2
    ;; uint16 _reference_id
    2
    ;; uint16 _corr_age
    2
    ;; uint32 _signal_info
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _nr_sv
       (write-byte _nr_sv s)
     ;; uint8 _error
       (write-byte _error s)
     ;; uint8 _mode
       (write-byte _mode s)
     ;; uint8 _misc
       (write-byte _misc s)
     ;; float64 _delta_x
       (sys::poke _delta_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_y
       (sys::poke _delta_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_z
       (sys::poke _delta_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _delta_vx
       (sys::poke _delta_vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _delta_vy
       (sys::poke _delta_vy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _delta_vz
       (sys::poke _delta_vz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _azimuth
       (write-word _azimuth s)
     ;; int16 _elevation
       (write-word _elevation s)
     ;; uint16 _reference_id
       (write-word _reference_id s)
     ;; uint16 _corr_age
       (write-word _corr_age s)
     ;; uint32 _signal_info
       (write-long _signal_info s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _nr_sv
     (setq _nr_sv (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _error
     (setq _error (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _misc
     (setq _misc (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _delta_x
     (setq _delta_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_y
     (setq _delta_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_z
     (setq _delta_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _delta_vx
     (setq _delta_vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _delta_vy
     (setq _delta_vy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _delta_vz
     (setq _delta_vz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _azimuth
     (setq _azimuth (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _elevation
     (setq _elevation (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _reference_id
     (setq _reference_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _corr_age
     (setq _corr_age (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _signal_info
     (setq _signal_info (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::VectorInfoCart :md5sum-) "d072946ecd7f4319790a07cf534d27c3")
(setf (get septentrio_gnss_driver::VectorInfoCart :datatype-) "septentrio_gnss_driver/VectorInfoCart")
(setf (get septentrio_gnss_driver::VectorInfoCart :definition-)
      "# VectorInfoCart block
   
uint8   nr_sv
uint8   error
uint8   mode
uint8   misc
float64 delta_x      # m
float64 delta_y      # m
float64 delta_z      # m
float32 delta_vx     # m
float32 delta_vy     # m
float32 delta_vz     # m
uint16  azimuth      # 0.01 deg
int16   elevation    # 0.01 deg
uint16  reference_id
uint16  corr_age     # 0.01 s
uint32  signal_info
")



(provide :septentrio_gnss_driver/VectorInfoCart "d072946ecd7f4319790a07cf534d27c3")


