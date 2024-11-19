;; Auto-generated. Do not edit!


(when (boundp 'septentrio_gnss_driver::PVTGeodetic)
  (if (not (find-package "SEPTENTRIO_GNSS_DRIVER"))
    (make-package "SEPTENTRIO_GNSS_DRIVER"))
  (shadow 'PVTGeodetic (find-package "SEPTENTRIO_GNSS_DRIVER")))
(unless (find-package "SEPTENTRIO_GNSS_DRIVER::PVTGEODETIC")
  (make-package "SEPTENTRIO_GNSS_DRIVER::PVTGEODETIC"))

(in-package "ROS")
;;//! \htmlinclude PVTGeodetic.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass septentrio_gnss_driver::PVTGeodetic
  :super ros::object
  :slots (_header _block_header _mode _error _latitude _longitude _height _undulation _vn _ve _vu _cog _rx_clk_bias _rx_clk_drift _time_system _datum _nr_sv _wa_corr_info _reference_id _mean_corr_age _signal_info _alert_flag _nr_bases _ppp_info _latency _h_accuracy _v_accuracy _misc ))

(defmethod septentrio_gnss_driver::PVTGeodetic
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:block_header __block_header) (instance septentrio_gnss_driver::BlockHeader :init))
    ((:mode __mode) 0)
    ((:error __error) 0)
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:height __height) 0.0)
    ((:undulation __undulation) 0.0)
    ((:vn __vn) 0.0)
    ((:ve __ve) 0.0)
    ((:vu __vu) 0.0)
    ((:cog __cog) 0.0)
    ((:rx_clk_bias __rx_clk_bias) 0.0)
    ((:rx_clk_drift __rx_clk_drift) 0.0)
    ((:time_system __time_system) 0)
    ((:datum __datum) 0)
    ((:nr_sv __nr_sv) 0)
    ((:wa_corr_info __wa_corr_info) 0)
    ((:reference_id __reference_id) 0)
    ((:mean_corr_age __mean_corr_age) 0)
    ((:signal_info __signal_info) 0)
    ((:alert_flag __alert_flag) 0)
    ((:nr_bases __nr_bases) 0)
    ((:ppp_info __ppp_info) 0)
    ((:latency __latency) 0)
    ((:h_accuracy __h_accuracy) 0)
    ((:v_accuracy __v_accuracy) 0)
    ((:misc __misc) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _block_header __block_header)
   (setq _mode (round __mode))
   (setq _error (round __error))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _height (float __height))
   (setq _undulation (float __undulation))
   (setq _vn (float __vn))
   (setq _ve (float __ve))
   (setq _vu (float __vu))
   (setq _cog (float __cog))
   (setq _rx_clk_bias (float __rx_clk_bias))
   (setq _rx_clk_drift (float __rx_clk_drift))
   (setq _time_system (round __time_system))
   (setq _datum (round __datum))
   (setq _nr_sv (round __nr_sv))
   (setq _wa_corr_info (round __wa_corr_info))
   (setq _reference_id (round __reference_id))
   (setq _mean_corr_age (round __mean_corr_age))
   (setq _signal_info (round __signal_info))
   (setq _alert_flag (round __alert_flag))
   (setq _nr_bases (round __nr_bases))
   (setq _ppp_info (round __ppp_info))
   (setq _latency (round __latency))
   (setq _h_accuracy (round __h_accuracy))
   (setq _v_accuracy (round __v_accuracy))
   (setq _misc (round __misc))
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
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
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
  (:vn
   (&optional __vn)
   (if __vn (setq _vn __vn)) _vn)
  (:ve
   (&optional __ve)
   (if __ve (setq _ve __ve)) _ve)
  (:vu
   (&optional __vu)
   (if __vu (setq _vu __vu)) _vu)
  (:cog
   (&optional __cog)
   (if __cog (setq _cog __cog)) _cog)
  (:rx_clk_bias
   (&optional __rx_clk_bias)
   (if __rx_clk_bias (setq _rx_clk_bias __rx_clk_bias)) _rx_clk_bias)
  (:rx_clk_drift
   (&optional __rx_clk_drift)
   (if __rx_clk_drift (setq _rx_clk_drift __rx_clk_drift)) _rx_clk_drift)
  (:time_system
   (&optional __time_system)
   (if __time_system (setq _time_system __time_system)) _time_system)
  (:datum
   (&optional __datum)
   (if __datum (setq _datum __datum)) _datum)
  (:nr_sv
   (&optional __nr_sv)
   (if __nr_sv (setq _nr_sv __nr_sv)) _nr_sv)
  (:wa_corr_info
   (&optional __wa_corr_info)
   (if __wa_corr_info (setq _wa_corr_info __wa_corr_info)) _wa_corr_info)
  (:reference_id
   (&optional __reference_id)
   (if __reference_id (setq _reference_id __reference_id)) _reference_id)
  (:mean_corr_age
   (&optional __mean_corr_age)
   (if __mean_corr_age (setq _mean_corr_age __mean_corr_age)) _mean_corr_age)
  (:signal_info
   (&optional __signal_info)
   (if __signal_info (setq _signal_info __signal_info)) _signal_info)
  (:alert_flag
   (&optional __alert_flag)
   (if __alert_flag (setq _alert_flag __alert_flag)) _alert_flag)
  (:nr_bases
   (&optional __nr_bases)
   (if __nr_bases (setq _nr_bases __nr_bases)) _nr_bases)
  (:ppp_info
   (&optional __ppp_info)
   (if __ppp_info (setq _ppp_info __ppp_info)) _ppp_info)
  (:latency
   (&optional __latency)
   (if __latency (setq _latency __latency)) _latency)
  (:h_accuracy
   (&optional __h_accuracy)
   (if __h_accuracy (setq _h_accuracy __h_accuracy)) _h_accuracy)
  (:v_accuracy
   (&optional __v_accuracy)
   (if __v_accuracy (setq _v_accuracy __v_accuracy)) _v_accuracy)
  (:misc
   (&optional __misc)
   (if __misc (setq _misc __misc)) _misc)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; septentrio_gnss_driver/BlockHeader _block_header
    (send _block_header :serialization-length)
    ;; uint8 _mode
    1
    ;; uint8 _error
    1
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _height
    8
    ;; float32 _undulation
    4
    ;; float32 _vn
    4
    ;; float32 _ve
    4
    ;; float32 _vu
    4
    ;; float32 _cog
    4
    ;; float64 _rx_clk_bias
    8
    ;; float32 _rx_clk_drift
    4
    ;; uint8 _time_system
    1
    ;; uint8 _datum
    1
    ;; uint8 _nr_sv
    1
    ;; uint8 _wa_corr_info
    1
    ;; uint16 _reference_id
    2
    ;; uint16 _mean_corr_age
    2
    ;; uint32 _signal_info
    4
    ;; uint8 _alert_flag
    1
    ;; uint8 _nr_bases
    1
    ;; uint16 _ppp_info
    2
    ;; uint16 _latency
    2
    ;; uint16 _h_accuracy
    2
    ;; uint16 _v_accuracy
    2
    ;; uint8 _misc
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; septentrio_gnss_driver/BlockHeader _block_header
       (send _block_header :serialize s)
     ;; uint8 _mode
       (write-byte _mode s)
     ;; uint8 _error
       (write-byte _error s)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _undulation
       (sys::poke _undulation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vn
       (sys::poke _vn (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ve
       (sys::poke _ve (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vu
       (sys::poke _vu (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cog
       (sys::poke _cog (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _rx_clk_bias
       (sys::poke _rx_clk_bias (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _rx_clk_drift
       (sys::poke _rx_clk_drift (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _time_system
       (write-byte _time_system s)
     ;; uint8 _datum
       (write-byte _datum s)
     ;; uint8 _nr_sv
       (write-byte _nr_sv s)
     ;; uint8 _wa_corr_info
       (write-byte _wa_corr_info s)
     ;; uint16 _reference_id
       (write-word _reference_id s)
     ;; uint16 _mean_corr_age
       (write-word _mean_corr_age s)
     ;; uint32 _signal_info
       (write-long _signal_info s)
     ;; uint8 _alert_flag
       (write-byte _alert_flag s)
     ;; uint8 _nr_bases
       (write-byte _nr_bases s)
     ;; uint16 _ppp_info
       (write-word _ppp_info s)
     ;; uint16 _latency
       (write-word _latency s)
     ;; uint16 _h_accuracy
       (write-word _h_accuracy s)
     ;; uint16 _v_accuracy
       (write-word _v_accuracy s)
     ;; uint8 _misc
       (write-byte _misc s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; septentrio_gnss_driver/BlockHeader _block_header
     (send _block_header :deserialize buf ptr-) (incf ptr- (send _block_header :serialization-length))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _error
     (setq _error (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _undulation
     (setq _undulation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vn
     (setq _vn (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ve
     (setq _ve (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vu
     (setq _vu (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cog
     (setq _cog (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _rx_clk_bias
     (setq _rx_clk_bias (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _rx_clk_drift
     (setq _rx_clk_drift (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _time_system
     (setq _time_system (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _datum
     (setq _datum (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _nr_sv
     (setq _nr_sv (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _wa_corr_info
     (setq _wa_corr_info (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _reference_id
     (setq _reference_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _mean_corr_age
     (setq _mean_corr_age (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _signal_info
     (setq _signal_info (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _alert_flag
     (setq _alert_flag (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _nr_bases
     (setq _nr_bases (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _ppp_info
     (setq _ppp_info (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _latency
     (setq _latency (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _h_accuracy
     (setq _h_accuracy (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _v_accuracy
     (setq _v_accuracy (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _misc
     (setq _misc (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get septentrio_gnss_driver::PVTGeodetic :md5sum-) "4d3911a0efd862e13c076ba7d322cd3e")
(setf (get septentrio_gnss_driver::PVTGeodetic :datatype-) "septentrio_gnss_driver/PVTGeodetic")
(setf (get septentrio_gnss_driver::PVTGeodetic :definition-)
      "# PVTGeodetic block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float64      latitude      # rad    
float64      longitude     # rad
float64      height        # m (ellipsoidal)
float32      undulation    # m
float32      vn            # m/s
float32      ve            # m/s
float32      vu            # m/s
float32      cog           # deg
float64      rx_clk_bias   # ms
float32      rx_clk_drift  # ppm
uint8        time_system 
uint8        datum
uint8        nr_sv 
uint8        wa_corr_info
uint16       reference_id
uint16       mean_corr_age # 0.01s
uint32       signal_info
uint8        alert_flag
uint8        nr_bases
uint16       ppp_info      # s
uint16       latency       # 0.0001 s
uint16       h_accuracy    # 0.01 m
uint16       v_accuracy    # 0.01 m
uint8        misc
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



(provide :septentrio_gnss_driver/PVTGeodetic "4d3911a0efd862e13c076ba7d322cd3e")


