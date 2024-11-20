;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::VectorInfoGeod)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'VectorInfoGeod (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::VECTORINFOGEOD")
  (make-package "SEPTENTRIO_GNSS_DRIVER::VECTORINFOGEOD"))

(in-package "ROS")
;;//! \htmlinclude VectorInfoGeod.msg.html


(defclass septentrio_gnss_driver::VectorInfoGeod
  :super ros::object
  :slots (_nr_sv _error _mode _misc _delta_east _delta_north _delta_up _delta_ve _delta_vn _delta_vu _azimuth _elevation _reference_id _corr_age _signal_info ))

(defmethod septentrio_gnss_driver::VectorInfoGeod
  (:init
   (&key
    ((:nr_sv __nr_sv) 0)
    ((:error __error) 0)
    ((:mode __mode) 0)
    ((:misc __misc) 0)
    ((:delta_east __delta_east) 0.0)
    ((:delta_north __delta_north) 0.0)
    ((:delta_up __delta_up) 0.0)
    ((:delta_ve __delta_ve) 0.0)
    ((:delta_vn __delta_vn) 0.0)
    ((:delta_vu __delta_vu) 0.0)
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
   (setq _delta_east (float __delta_east))
   (setq _delta_north (float __delta_north))
   (setq _delta_up (float __delta_up))
   (setq _delta_ve (float __delta_ve))
   (setq _delta_vn (float __delta_vn))
   (setq _delta_vu (float __delta_vu))
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
  (:delta_east
   (&optional __delta_east)
   (if __delta_east (setq _delta_east __delta_east)) _delta_east)
  (:delta_north
   (&optional __delta_north)
   (if __delta_north (setq _delta_north __delta_north)) _delta_north)
  (:delta_up
   (&optional __delta_up)
   (if __delta_up (setq _delta_up __delta_up)) _delta_up)
  (:delta_ve
   (&optional __delta_ve)
   (if __delta_ve (setq _delta_ve __delta_ve)) _delta_ve)
  (:delta_vn
   (&optional __delta_vn)
   (if __delta_vn (setq _delta_vn __delta_vn)) _delta_vn)
  (:delta_vu
   (&optional __delta_vu)
   (if __delta_vu (setq _delta_vu __delta_vu)) _delta_vu)
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
    ;; float64 _delta_east
    8
    ;; float64 _delta_north
    8
    ;; float64 _delta_up
    8
    ;; float32 _delta_ve
    4
    ;; float32 _delta_vn
    4
    ;; float32 _delta_vu
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
     ;; float64 _delta_east
       (sys::poke _delta_east (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_north
       (sys::poke _delta_north (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_up
       (sys::poke _delta_up (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _delta_ve
       (sys::poke _delta_ve (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _delta_vn
       (sys::poke _delta_vn (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _delta_vu
       (sys::poke _delta_vu (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float64 _delta_east
     (setq _delta_east (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_north
     (setq _delta_north (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_up
     (setq _delta_up (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _delta_ve
     (setq _delta_ve (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _delta_vn
     (setq _delta_vn (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _delta_vu
     (setq _delta_vu (sys::peek buf ptr- :float)) (incf ptr- 4)
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

(setf (get septentrio_gnss_driver::VectorInfoGeod :md5sum-) "dbd3c8ede9fdca23023a59b11ba2b072")
(setf (get septentrio_gnss_driver::VectorInfoGeod :datatype-) "septentrio_gnss_driver/VectorInfoGeod")
(setf (get septentrio_gnss_driver::VectorInfoGeod :definition-)
      "# VectorInfoGeod block
   
uint8   nr_sv
uint8   error
uint8   mode
uint8   misc
float64 delta_east   # m
float64 delta_north  # m
float64 delta_up     # m
float32 delta_ve     # m
float32 delta_vn     # m
float32 delta_vu     # m
uint16  azimuth      # 0.01 deg
int16   elevation    # 0.01 deg
uint16  reference_id
uint16  corr_age     # 0.01 s
uint32  signal_info
")



(provide :septentrio_gnss_driver/VectorInfoGeod "dbd3c8ede9fdca23023a59b11ba2b072")


