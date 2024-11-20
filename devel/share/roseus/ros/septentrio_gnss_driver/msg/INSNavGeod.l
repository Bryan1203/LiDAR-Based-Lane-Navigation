;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::INSNavGeod)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'INSNavGeod (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::INSNAVGEOD")
  (make-package "SEPTENTRIO_GNSS_DRIVER::INSNAVGEOD"))

(in-package "ROS")
;;//! \htmlinclude INSNavGeod.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::INSNavGeod
  :super ros::object
  :slots (_header _block_header _gnss_mode _error _info _gnss_age _latitude _longitude _height _undulation _accuracy _latency _datum _sb_list _latitude_std_dev _longitude_std_dev _height_std_dev _latitude_longitude_cov _latitude_height_cov _longitude_height_cov _heading _pitch _roll _heading_std_dev _pitch_std_dev _roll_std_dev _heading_pitch_cov _heading_roll_cov _pitch_roll_cov _ve _vn _vu _ve_std_dev _vn_std_dev _vu_std_dev _ve_vn_cov _ve_vu_cov _vn_vu_cov ))

(defmethod septentrio_gnss_driver::INSNavGeod
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:gnss_mode __gnss_mode) 0)
    ((:error __error) 0)
    ((:info __info) 0)
    ((:gnss_age __gnss_age) 0)
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:height __height) 0.0)
    ((:undulation __undulation) 0.0)
    ((:accuracy __accuracy) 0)
    ((:latency __latency) 0)
    ((:datum __datum) 0)
    ((:sb_list __sb_list) 0)
    ((:latitude_std_dev __latitude_std_dev) 0.0)
    ((:longitude_std_dev __longitude_std_dev) 0.0)
    ((:height_std_dev __height_std_dev) 0.0)
    ((:latitude_longitude_cov __latitude_longitude_cov) 0.0)
    ((:latitude_height_cov __latitude_height_cov) 0.0)
    ((:longitude_height_cov __longitude_height_cov) 0.0)
    ((:heading __heading) 0.0)
    ((:pitch __pitch) 0.0)
    ((:roll __roll) 0.0)
    ((:heading_std_dev __heading_std_dev) 0.0)
    ((:pitch_std_dev __pitch_std_dev) 0.0)
    ((:roll_std_dev __roll_std_dev) 0.0)
    ((:heading_pitch_cov __heading_pitch_cov) 0.0)
    ((:heading_roll_cov __heading_roll_cov) 0.0)
    ((:pitch_roll_cov __pitch_roll_cov) 0.0)
    ((:ve __ve) 0.0)
    ((:vn __vn) 0.0)
    ((:vu __vu) 0.0)
    ((:ve_std_dev __ve_std_dev) 0.0)
    ((:vn_std_dev __vn_std_dev) 0.0)
    ((:vu_std_dev __vu_std_dev) 0.0)
    ((:ve_vn_cov __ve_vn_cov) 0.0)
    ((:ve_vu_cov __ve_vu_cov) 0.0)
    ((:vn_vu_cov __vn_vu_cov) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _gnss_mode (round __gnss_mode))
   (setq _error (round __error))
   (setq _info (round __info))
   (setq _gnss_age (round __gnss_age))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _height (float __height))
   (setq _undulation (float __undulation))
   (setq _accuracy (round __accuracy))
   (setq _latency (round __latency))
   (setq _datum (round __datum))
   (setq _sb_list (round __sb_list))
   (setq _latitude_std_dev (float __latitude_std_dev))
   (setq _longitude_std_dev (float __longitude_std_dev))
   (setq _height_std_dev (float __height_std_dev))
   (setq _latitude_longitude_cov (float __latitude_longitude_cov))
   (setq _latitude_height_cov (float __latitude_height_cov))
   (setq _longitude_height_cov (float __longitude_height_cov))
   (setq _heading (float __heading))
   (setq _pitch (float __pitch))
   (setq _roll (float __roll))
   (setq _heading_std_dev (float __heading_std_dev))
   (setq _pitch_std_dev (float __pitch_std_dev))
   (setq _roll_std_dev (float __roll_std_dev))
   (setq _heading_pitch_cov (float __heading_pitch_cov))
   (setq _heading_roll_cov (float __heading_roll_cov))
   (setq _pitch_roll_cov (float __pitch_roll_cov))
   (setq _ve (float __ve))
   (setq _vn (float __vn))
   (setq _vu (float __vu))
   (setq _ve_std_dev (float __ve_std_dev))
   (setq _vn_std_dev (float __vn_std_dev))
   (setq _vu_std_dev (float __vu_std_dev))
   (setq _ve_vn_cov (float __ve_vn_cov))
   (setq _ve_vu_cov (float __ve_vu_cov))
   (setq _vn_vu_cov (float __vn_vu_cov))
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
  (:gnss_mode
   (&optional __gnss_mode)
   (if __gnss_mode (setq _gnss_mode __gnss_mode)) _gnss_mode)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:gnss_age
   (&optional __gnss_age)
   (if __gnss_age (setq _gnss_age __gnss_age)) _gnss_age)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:undulation
   (&optional __undulation)
   (if __undulation (setq _undulation __undulation)) _undulation)
  (:accuracy
   (&optional __accuracy)
   (if __accuracy (setq _accuracy __accuracy)) _accuracy)
  (:latency
   (&optional __latency)
   (if __latency (setq _latency __latency)) _latency)
  (:datum
   (&optional __datum)
   (if __datum (setq _datum __datum)) _datum)
  (:sb_list
   (&optional __sb_list)
   (if __sb_list (setq _sb_list __sb_list)) _sb_list)
  (:latitude_std_dev
   (&optional __latitude_std_dev)
   (if __latitude_std_dev (setq _latitude_std_dev __latitude_std_dev)) _latitude_std_dev)
  (:longitude_std_dev
   (&optional __longitude_std_dev)
   (if __longitude_std_dev (setq _longitude_std_dev __longitude_std_dev)) _longitude_std_dev)
  (:height_std_dev
   (&optional __height_std_dev)
   (if __height_std_dev (setq _height_std_dev __height_std_dev)) _height_std_dev)
  (:latitude_longitude_cov
   (&optional __latitude_longitude_cov)
   (if __latitude_longitude_cov (setq _latitude_longitude_cov __latitude_longitude_cov)) _latitude_longitude_cov)
  (:latitude_height_cov
   (&optional __latitude_height_cov)
   (if __latitude_height_cov (setq _latitude_height_cov __latitude_height_cov)) _latitude_height_cov)
  (:longitude_height_cov
   (&optional __longitude_height_cov)
   (if __longitude_height_cov (setq _longitude_height_cov __longitude_height_cov)) _longitude_height_cov)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:heading_std_dev
   (&optional __heading_std_dev)
   (if __heading_std_dev (setq _heading_std_dev __heading_std_dev)) _heading_std_dev)
  (:pitch_std_dev
   (&optional __pitch_std_dev)
   (if __pitch_std_dev (setq _pitch_std_dev __pitch_std_dev)) _pitch_std_dev)
  (:roll_std_dev
   (&optional __roll_std_dev)
   (if __roll_std_dev (setq _roll_std_dev __roll_std_dev)) _roll_std_dev)
  (:heading_pitch_cov
   (&optional __heading_pitch_cov)
   (if __heading_pitch_cov (setq _heading_pitch_cov __heading_pitch_cov)) _heading_pitch_cov)
  (:heading_roll_cov
   (&optional __heading_roll_cov)
   (if __heading_roll_cov (setq _heading_roll_cov __heading_roll_cov)) _heading_roll_cov)
  (:pitch_roll_cov
   (&optional __pitch_roll_cov)
   (if __pitch_roll_cov (setq _pitch_roll_cov __pitch_roll_cov)) _pitch_roll_cov)
  (:ve
   (&optional __ve)
   (if __ve (setq _ve __ve)) _ve)
  (:vn
   (&optional __vn)
   (if __vn (setq _vn __vn)) _vn)
  (:vu
   (&optional __vu)
   (if __vu (setq _vu __vu)) _vu)
  (:ve_std_dev
   (&optional __ve_std_dev)
   (if __ve_std_dev (setq _ve_std_dev __ve_std_dev)) _ve_std_dev)
  (:vn_std_dev
   (&optional __vn_std_dev)
   (if __vn_std_dev (setq _vn_std_dev __vn_std_dev)) _vn_std_dev)
  (:vu_std_dev
   (&optional __vu_std_dev)
   (if __vu_std_dev (setq _vu_std_dev __vu_std_dev)) _vu_std_dev)
  (:ve_vn_cov
   (&optional __ve_vn_cov)
   (if __ve_vn_cov (setq _ve_vn_cov __ve_vn_cov)) _ve_vn_cov)
  (:ve_vu_cov
   (&optional __ve_vu_cov)
   (if __ve_vu_cov (setq _ve_vu_cov __ve_vu_cov)) _ve_vu_cov)
  (:vn_vu_cov
   (&optional __vn_vu_cov)
   (if __vn_vu_cov (setq _vn_vu_cov __vn_vu_cov)) _vn_vu_cov)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; septentrio_gnss_driver/BlockHeader _block_header
    (send _block_header :serialization-length)
    ;; uint8 _gnss_mode
    1
    ;; uint8 _error
    1
    ;; uint16 _info
    2
    ;; uint16 _gnss_age
    2
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _height
    8
    ;; float32 _undulation
    4
    ;; uint16 _accuracy
    2
    ;; uint16 _latency
    2
    ;; uint8 _datum
    1
    ;; uint16 _sb_list
    2
    ;; float32 _latitude_std_dev
    4
    ;; float32 _longitude_std_dev
    4
    ;; float32 _height_std_dev
    4
    ;; float32 _latitude_longitude_cov
    4
    ;; float32 _latitude_height_cov
    4
    ;; float32 _longitude_height_cov
    4
    ;; float32 _heading
    4
    ;; float32 _pitch
    4
    ;; float32 _roll
    4
    ;; float32 _heading_std_dev
    4
    ;; float32 _pitch_std_dev
    4
    ;; float32 _roll_std_dev
    4
    ;; float32 _heading_pitch_cov
    4
    ;; float32 _heading_roll_cov
    4
    ;; float32 _pitch_roll_cov
    4
    ;; float32 _ve
    4
    ;; float32 _vn
    4
    ;; float32 _vu
    4
    ;; float32 _ve_std_dev
    4
    ;; float32 _vn_std_dev
    4
    ;; float32 _vu_std_dev
    4
    ;; float32 _ve_vn_cov
    4
    ;; float32 _ve_vu_cov
    4
    ;; float32 _vn_vu_cov
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; septentrio_gnss_driver/BlockHeader _block_header
       (send _block_header :serialize s)
     ;; uint8 _gnss_mode
       (write-byte _gnss_mode s)
     ;; uint8 _error
       (write-byte _error s)
     ;; uint16 _info
       (write-word _info s)
     ;; uint16 _gnss_age
       (write-word _gnss_age s)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _undulation
       (sys::poke _undulation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _accuracy
       (write-word _accuracy s)
     ;; uint16 _latency
       (write-word _latency s)
     ;; uint8 _datum
       (write-byte _datum s)
     ;; uint16 _sb_list
       (write-word _sb_list s)
     ;; float32 _latitude_std_dev
       (sys::poke _latitude_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _longitude_std_dev
       (sys::poke _longitude_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height_std_dev
       (sys::poke _height_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _latitude_longitude_cov
       (sys::poke _latitude_longitude_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _latitude_height_cov
       (sys::poke _latitude_height_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _longitude_height_cov
       (sys::poke _longitude_height_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_std_dev
       (sys::poke _heading_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_std_dev
       (sys::poke _pitch_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_std_dev
       (sys::poke _roll_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_pitch_cov
       (sys::poke _heading_pitch_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_roll_cov
       (sys::poke _heading_roll_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_roll_cov
       (sys::poke _pitch_roll_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ve
       (sys::poke _ve (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vn
       (sys::poke _vn (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vu
       (sys::poke _vu (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ve_std_dev
       (sys::poke _ve_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vn_std_dev
       (sys::poke _vn_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vu_std_dev
       (sys::poke _vu_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ve_vn_cov
       (sys::poke _ve_vn_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ve_vu_cov
       (sys::poke _ve_vu_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vn_vu_cov
       (sys::poke _vn_vu_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; septentrio_gnss_driver/BlockHeader _block_header
     (send _block_header :deserialize buf ptr-) (incf ptr- (send _block_header :serialization-length))
   ;; uint8 _gnss_mode
     (setq _gnss_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _error
     (setq _error (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _info
     (setq _info (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _gnss_age
     (setq _gnss_age (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _undulation
     (setq _undulation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _accuracy
     (setq _accuracy (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _latency
     (setq _latency (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _datum
     (setq _datum (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _sb_list
     (setq _sb_list (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _latitude_std_dev
     (setq _latitude_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _longitude_std_dev
     (setq _longitude_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height_std_dev
     (setq _height_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _latitude_longitude_cov
     (setq _latitude_longitude_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _latitude_height_cov
     (setq _latitude_height_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _longitude_height_cov
     (setq _longitude_height_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading
     (setq _heading (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_std_dev
     (setq _heading_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_std_dev
     (setq _pitch_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_std_dev
     (setq _roll_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_pitch_cov
     (setq _heading_pitch_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_roll_cov
     (setq _heading_roll_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_roll_cov
     (setq _pitch_roll_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ve
     (setq _ve (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vn
     (setq _vn (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vu
     (setq _vu (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ve_std_dev
     (setq _ve_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vn_std_dev
     (setq _vn_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vu_std_dev
     (setq _vu_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ve_vn_cov
     (setq _ve_vn_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ve_vu_cov
     (setq _ve_vu_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vn_vu_cov
     (setq _vn_vu_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::INSNavGeod :md5sum-) "7e95cd9c24ef355a547323f4ca3d6e14")
(setf (get septentrio_gnss_driver::INSNavGeod :datatype-) "septentrio_gnss_driver/INSNavGeod")
(setf (get septentrio_gnss_driver::INSNavGeod :definition-)
      "# INSNavGeod block 
# Block_Number 4226

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint8       gnss_mode
uint8       error
uint16      info
uint16      gnss_age   # 0.01 s
float64     latitude   # rad
float64     longitude  # rad
float64     height     # m (ellipsoidal)
float32     undulation # m
uint16      accuracy   # 0.01 m
uint16      latency    # 0.0001 s
uint8       datum
#uint8       reserved
uint16      sb_list

# INSNavGeodPosStdDev sub-block definition:
# If the Position StdDev field is 1 then this sub block is available:
float32     latitude_std_dev  # m
float32     longitude_std_dev # m
float32     height_std_dev    # m

# INSNavGeodPosCov sub-block definition:
# If the Position Cov field is 1 then this sub block is available:
float32     latitude_longitude_cov # m^2
float32     latitude_height_cov    # m^2
float32     longitude_height_cov   # m^2

# INSNavGeodAtt sub-block definition:
# If the Attitude field is 1 then this sub block is available:
float32     heading # deg
float32     pitch   # deg
float32     roll    # deg

# INSNavGeodAttStdDev sub-block definition:
# If the Attitude StdDev field is 1 then this sub block is available:
float32     heading_std_dev # deg
float32     pitch_std_dev   # deg
float32     roll_std_dev    # deg

# INSNavGeodAttCov sub-block definition:
# If the Attitude Cov field is 1 then this sub block is available:
float32     heading_pitch_cov # deg^2
float32     heading_roll_cov  # deg^2
float32     pitch_roll_cov    # deg^2

# INSNavGeodVel sub-block definition:
# If the Velocity field is 1 then this sub block is available:
float32     ve # m/s
float32     vn # m/s 
float32     vu # m/s

# INSNavGeodVelStdDev sub-block definition:
# If the Velocity StdDev field is 1 then this sub block is available:
float32     ve_std_dev # m/s
float32     vn_std_dev # m/s
float32     vu_std_dev # m/s

# INSNavGeodVelCov sub-block definition:
# If the Velocity Cov field is 1 then this sub block is available:
float32     ve_vn_cov # m^2/s^2
float32     ve_vu_cov # m^2/s^2
float32     vn_vu_cov # m^2/s^2   

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



(provide :septentrio_gnss_driver/INSNavGeod "7e95cd9c24ef355a547323f4ca3d6e14")


