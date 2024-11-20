;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::INSNavCart)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'INSNavCart (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::INSNAVCART")
  (make-package "SEPTENTRIO_GNSS_DRIVER::INSNAVCART"))

(in-package "ROS")
;;//! \htmlinclude INSNavCart.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::INSNavCart
  :super ros::object
  :slots (_header _block_header _gnss_mode _error _info _gnss_age _x _y _z _accuracy _latency _datum _sb_list _x_std_dev _y_std_dev _z_std_dev _xy_cov _xz_cov _yz_cov _heading _pitch _roll _heading_std_dev _pitch_std_dev _roll_std_dev _heading_pitch_cov _heading_roll_cov _pitch_roll_cov _vx _vy _vz _vx_std_dev _vy_std_dev _vz_std_dev _vx_vy_cov _vx_vz_cov _vy_vz_cov ))

(defmethod septentrio_gnss_driver::INSNavCart
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:gnss_mode __gnss_mode) 0)
    ((:error __error) 0)
    ((:info __info) 0)
    ((:gnss_age __gnss_age) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:accuracy __accuracy) 0)
    ((:latency __latency) 0)
    ((:datum __datum) 0)
    ((:sb_list __sb_list) 0)
    ((:x_std_dev __x_std_dev) 0.0)
    ((:y_std_dev __y_std_dev) 0.0)
    ((:z_std_dev __z_std_dev) 0.0)
    ((:xy_cov __xy_cov) 0.0)
    ((:xz_cov __xz_cov) 0.0)
    ((:yz_cov __yz_cov) 0.0)
    ((:heading __heading) 0.0)
    ((:pitch __pitch) 0.0)
    ((:roll __roll) 0.0)
    ((:heading_std_dev __heading_std_dev) 0.0)
    ((:pitch_std_dev __pitch_std_dev) 0.0)
    ((:roll_std_dev __roll_std_dev) 0.0)
    ((:heading_pitch_cov __heading_pitch_cov) 0.0)
    ((:heading_roll_cov __heading_roll_cov) 0.0)
    ((:pitch_roll_cov __pitch_roll_cov) 0.0)
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:vz __vz) 0.0)
    ((:vx_std_dev __vx_std_dev) 0.0)
    ((:vy_std_dev __vy_std_dev) 0.0)
    ((:vz_std_dev __vz_std_dev) 0.0)
    ((:vx_vy_cov __vx_vy_cov) 0.0)
    ((:vx_vz_cov __vx_vz_cov) 0.0)
    ((:vy_vz_cov __vy_vz_cov) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _gnss_mode (round __gnss_mode))
   (setq _error (round __error))
   (setq _info (round __info))
   (setq _gnss_age (round __gnss_age))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _accuracy (round __accuracy))
   (setq _latency (round __latency))
   (setq _datum (round __datum))
   (setq _sb_list (round __sb_list))
   (setq _x_std_dev (float __x_std_dev))
   (setq _y_std_dev (float __y_std_dev))
   (setq _z_std_dev (float __z_std_dev))
   (setq _xy_cov (float __xy_cov))
   (setq _xz_cov (float __xz_cov))
   (setq _yz_cov (float __yz_cov))
   (setq _heading (float __heading))
   (setq _pitch (float __pitch))
   (setq _roll (float __roll))
   (setq _heading_std_dev (float __heading_std_dev))
   (setq _pitch_std_dev (float __pitch_std_dev))
   (setq _roll_std_dev (float __roll_std_dev))
   (setq _heading_pitch_cov (float __heading_pitch_cov))
   (setq _heading_roll_cov (float __heading_roll_cov))
   (setq _pitch_roll_cov (float __pitch_roll_cov))
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _vz (float __vz))
   (setq _vx_std_dev (float __vx_std_dev))
   (setq _vy_std_dev (float __vy_std_dev))
   (setq _vz_std_dev (float __vz_std_dev))
   (setq _vx_vy_cov (float __vx_vy_cov))
   (setq _vx_vz_cov (float __vx_vz_cov))
   (setq _vy_vz_cov (float __vy_vz_cov))
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
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
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
  (:x_std_dev
   (&optional __x_std_dev)
   (if __x_std_dev (setq _x_std_dev __x_std_dev)) _x_std_dev)
  (:y_std_dev
   (&optional __y_std_dev)
   (if __y_std_dev (setq _y_std_dev __y_std_dev)) _y_std_dev)
  (:z_std_dev
   (&optional __z_std_dev)
   (if __z_std_dev (setq _z_std_dev __z_std_dev)) _z_std_dev)
  (:xy_cov
   (&optional __xy_cov)
   (if __xy_cov (setq _xy_cov __xy_cov)) _xy_cov)
  (:xz_cov
   (&optional __xz_cov)
   (if __xz_cov (setq _xz_cov __xz_cov)) _xz_cov)
  (:yz_cov
   (&optional __yz_cov)
   (if __yz_cov (setq _yz_cov __yz_cov)) _yz_cov)
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
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:vz
   (&optional __vz)
   (if __vz (setq _vz __vz)) _vz)
  (:vx_std_dev
   (&optional __vx_std_dev)
   (if __vx_std_dev (setq _vx_std_dev __vx_std_dev)) _vx_std_dev)
  (:vy_std_dev
   (&optional __vy_std_dev)
   (if __vy_std_dev (setq _vy_std_dev __vy_std_dev)) _vy_std_dev)
  (:vz_std_dev
   (&optional __vz_std_dev)
   (if __vz_std_dev (setq _vz_std_dev __vz_std_dev)) _vz_std_dev)
  (:vx_vy_cov
   (&optional __vx_vy_cov)
   (if __vx_vy_cov (setq _vx_vy_cov __vx_vy_cov)) _vx_vy_cov)
  (:vx_vz_cov
   (&optional __vx_vz_cov)
   (if __vx_vz_cov (setq _vx_vz_cov __vx_vz_cov)) _vx_vz_cov)
  (:vy_vz_cov
   (&optional __vy_vz_cov)
   (if __vy_vz_cov (setq _vy_vz_cov __vy_vz_cov)) _vy_vz_cov)
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
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; uint16 _accuracy
    2
    ;; uint16 _latency
    2
    ;; uint8 _datum
    1
    ;; uint16 _sb_list
    2
    ;; float32 _x_std_dev
    4
    ;; float32 _y_std_dev
    4
    ;; float32 _z_std_dev
    4
    ;; float32 _xy_cov
    4
    ;; float32 _xz_cov
    4
    ;; float32 _yz_cov
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
    ;; float32 _vx
    4
    ;; float32 _vy
    4
    ;; float32 _vz
    4
    ;; float32 _vx_std_dev
    4
    ;; float32 _vy_std_dev
    4
    ;; float32 _vz_std_dev
    4
    ;; float32 _vx_vy_cov
    4
    ;; float32 _vx_vz_cov
    4
    ;; float32 _vy_vz_cov
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
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint16 _accuracy
       (write-word _accuracy s)
     ;; uint16 _latency
       (write-word _latency s)
     ;; uint8 _datum
       (write-byte _datum s)
     ;; uint16 _sb_list
       (write-word _sb_list s)
     ;; float32 _x_std_dev
       (sys::poke _x_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_std_dev
       (sys::poke _y_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_std_dev
       (sys::poke _z_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _xy_cov
       (sys::poke _xy_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _xz_cov
       (sys::poke _xz_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yz_cov
       (sys::poke _yz_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
     ;; float32 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vz
       (sys::poke _vz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vx_std_dev
       (sys::poke _vx_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vy_std_dev
       (sys::poke _vy_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vz_std_dev
       (sys::poke _vz_std_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vx_vy_cov
       (sys::poke _vx_vy_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vx_vz_cov
       (sys::poke _vx_vz_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vy_vz_cov
       (sys::poke _vy_vz_cov (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint16 _accuracy
     (setq _accuracy (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _latency
     (setq _latency (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _datum
     (setq _datum (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _sb_list
     (setq _sb_list (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _x_std_dev
     (setq _x_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_std_dev
     (setq _y_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_std_dev
     (setq _z_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _xy_cov
     (setq _xy_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _xz_cov
     (setq _xz_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yz_cov
     (setq _yz_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
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
   ;; float32 _vx
     (setq _vx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vy
     (setq _vy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vz
     (setq _vz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vx_std_dev
     (setq _vx_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vy_std_dev
     (setq _vy_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vz_std_dev
     (setq _vz_std_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vx_vy_cov
     (setq _vx_vy_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vx_vz_cov
     (setq _vx_vz_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vy_vz_cov
     (setq _vy_vz_cov (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::INSNavCart :md5sum-) "ebb1a1e03ea3fac47ad5f19264160123")
(setf (get septentrio_gnss_driver::INSNavCart :datatype-) "septentrio_gnss_driver/INSNavCart")
(setf (get septentrio_gnss_driver::INSNavCart :definition-)
      "# INSNavCart block 
# Block_Number 4225

std_msgs/Header header

# SBF block header including time header
BlockHeader block_header

uint8       gnss_mode
uint8       error
uint16      info
uint16      gnss_age # 0.01 s
float64     x        # m
float64     y        # m
float64     z        # m
uint16      accuracy # 0.01 m
uint16      latency  # 0.1 ms
uint8       datum    
#uint8       reserved
uint16      sb_list

# INSNavCartPosStdDev sub-block definition:
# If the Position StdDev field is 1 then this sub block is available:
float32     x_std_dev # m
float32     y_std_dev # m
float32     z_std_dev # m

# INSNavCartPosCov sub-block definition:
# If the Position Cov field is 1 then this sub block is available:
float32     xy_cov # m^2
float32     xz_cov # m^2
float32     yz_cov # m^2

# INSNavCartAtt sub-block definition:
# If the Attitude field is 1 then this sub block is available:
float32     heading # deg
float32     pitch   # deg
float32     roll    # deg

# INSNavCartAttStdDev sub-block definition:
# If the Attitude StdDev field is 1 then this sub block is available:
float32     heading_std_dev # deg
float32     pitch_std_dev   # deg
float32     roll_std_dev    # deg

# INSNavCartAttCov sub-block definition:
# If the Attitude Cov field is 1 then this sub block is available:
float32     heading_pitch_cov # deg^2
float32     heading_roll_cov  # deg^2
float32     pitch_roll_cov    # deg^2

# INSNavCartVel sub-block definition:
# If the Velocity field is 1 then this sub block is available:
float32     vx # m/s
float32     vy # m/S
float32     vz # m/s

# INSNavCartVelStdDev sub-block definition:
# If the Velocity StdDev field is 1 then this sub block is available:
float32     vx_std_dev # m/s
float32     vy_std_dev # m/s
float32     vz_std_dev # m/s

# INSNavCartVelCov sub-block definition:
# If the Velocity Cov field is 1 then this sub block is available:
float32     vx_vy_cov # m^2/s^2
float32     vx_vz_cov # m^2/s^2
float32     vy_vz_cov # m^2/s^2
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



(provide :septentrio_gnss_driver/INSNavCart "ebb1a1e03ea3fac47ad5f19264160123")


