;; Auto-generated. Do not edit!


(when (boundp 'camera_control_msgs::GetCamProperties)
  (if (not (find-package "CAMERA_CONTROL_MSGS"))
    (make-package "CAMERA_CONTROL_MSGS"))
  (shadow 'GetCamProperties (find-package "CAMERA_CONTROL_MSGS")))
(unless (find-package "CAMERA_CONTROL_MSGS::GETCAMPROPERTIES")
  (make-package "CAMERA_CONTROL_MSGS::GETCAMPROPERTIES"))
(unless (find-package "CAMERA_CONTROL_MSGS::GETCAMPROPERTIESREQUEST")
  (make-package "CAMERA_CONTROL_MSGS::GETCAMPROPERTIESREQUEST"))
(unless (find-package "CAMERA_CONTROL_MSGS::GETCAMPROPERTIESRESPONSE")
  (make-package "CAMERA_CONTROL_MSGS::GETCAMPROPERTIESRESPONSE"))

(in-package "ROS")





(defclass camera_control_msgs::GetCamPropertiesRequest
  :super ros::object
  :slots ())

(defmethod camera_control_msgs::GetCamPropertiesRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass camera_control_msgs::GetCamPropertiesResponse
  :super ros::object
  :slots (_success _is_sleeping _device_user_id _min_binning_x _max_binning_x _current_binning_x _min_binning_y _max_binning_y _current_binning_y _max_framerate _current_framerate _min_exposure _max_exposure _current_exposure _min_gain_in_cam_units _max_gain_in_cam_units _current_gain_in_cam_units _min_gain _max_gain _current_gain _min_gamma _max_gamma _current_gamma _brightness_continuous _gain_auto _exposure_auto ))

(defmethod camera_control_msgs::GetCamPropertiesResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:is_sleeping __is_sleeping) nil)
    ((:device_user_id __device_user_id) "")
    ((:min_binning_x __min_binning_x) 0)
    ((:max_binning_x __max_binning_x) 0)
    ((:current_binning_x __current_binning_x) 0)
    ((:min_binning_y __min_binning_y) 0)
    ((:max_binning_y __max_binning_y) 0)
    ((:current_binning_y __current_binning_y) 0)
    ((:max_framerate __max_framerate) 0.0)
    ((:current_framerate __current_framerate) 0.0)
    ((:min_exposure __min_exposure) 0.0)
    ((:max_exposure __max_exposure) 0.0)
    ((:current_exposure __current_exposure) 0.0)
    ((:min_gain_in_cam_units __min_gain_in_cam_units) 0.0)
    ((:max_gain_in_cam_units __max_gain_in_cam_units) 0.0)
    ((:current_gain_in_cam_units __current_gain_in_cam_units) 0.0)
    ((:min_gain __min_gain) 0.0)
    ((:max_gain __max_gain) 0.0)
    ((:current_gain __current_gain) 0.0)
    ((:min_gamma __min_gamma) 0.0)
    ((:max_gamma __max_gamma) 0.0)
    ((:current_gamma __current_gamma) 0.0)
    ((:brightness_continuous __brightness_continuous) nil)
    ((:gain_auto __gain_auto) nil)
    ((:exposure_auto __exposure_auto) nil)
    )
   (send-super :init)
   (setq _success __success)
   (setq _is_sleeping __is_sleeping)
   (setq _device_user_id (string __device_user_id))
   (setq _min_binning_x (round __min_binning_x))
   (setq _max_binning_x (round __max_binning_x))
   (setq _current_binning_x (round __current_binning_x))
   (setq _min_binning_y (round __min_binning_y))
   (setq _max_binning_y (round __max_binning_y))
   (setq _current_binning_y (round __current_binning_y))
   (setq _max_framerate (float __max_framerate))
   (setq _current_framerate (float __current_framerate))
   (setq _min_exposure (float __min_exposure))
   (setq _max_exposure (float __max_exposure))
   (setq _current_exposure (float __current_exposure))
   (setq _min_gain_in_cam_units (float __min_gain_in_cam_units))
   (setq _max_gain_in_cam_units (float __max_gain_in_cam_units))
   (setq _current_gain_in_cam_units (float __current_gain_in_cam_units))
   (setq _min_gain (float __min_gain))
   (setq _max_gain (float __max_gain))
   (setq _current_gain (float __current_gain))
   (setq _min_gamma (float __min_gamma))
   (setq _max_gamma (float __max_gamma))
   (setq _current_gamma (float __current_gamma))
   (setq _brightness_continuous __brightness_continuous)
   (setq _gain_auto __gain_auto)
   (setq _exposure_auto __exposure_auto)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:is_sleeping
   (&optional (__is_sleeping :null))
   (if (not (eq __is_sleeping :null)) (setq _is_sleeping __is_sleeping)) _is_sleeping)
  (:device_user_id
   (&optional __device_user_id)
   (if __device_user_id (setq _device_user_id __device_user_id)) _device_user_id)
  (:min_binning_x
   (&optional __min_binning_x)
   (if __min_binning_x (setq _min_binning_x __min_binning_x)) _min_binning_x)
  (:max_binning_x
   (&optional __max_binning_x)
   (if __max_binning_x (setq _max_binning_x __max_binning_x)) _max_binning_x)
  (:current_binning_x
   (&optional __current_binning_x)
   (if __current_binning_x (setq _current_binning_x __current_binning_x)) _current_binning_x)
  (:min_binning_y
   (&optional __min_binning_y)
   (if __min_binning_y (setq _min_binning_y __min_binning_y)) _min_binning_y)
  (:max_binning_y
   (&optional __max_binning_y)
   (if __max_binning_y (setq _max_binning_y __max_binning_y)) _max_binning_y)
  (:current_binning_y
   (&optional __current_binning_y)
   (if __current_binning_y (setq _current_binning_y __current_binning_y)) _current_binning_y)
  (:max_framerate
   (&optional __max_framerate)
   (if __max_framerate (setq _max_framerate __max_framerate)) _max_framerate)
  (:current_framerate
   (&optional __current_framerate)
   (if __current_framerate (setq _current_framerate __current_framerate)) _current_framerate)
  (:min_exposure
   (&optional __min_exposure)
   (if __min_exposure (setq _min_exposure __min_exposure)) _min_exposure)
  (:max_exposure
   (&optional __max_exposure)
   (if __max_exposure (setq _max_exposure __max_exposure)) _max_exposure)
  (:current_exposure
   (&optional __current_exposure)
   (if __current_exposure (setq _current_exposure __current_exposure)) _current_exposure)
  (:min_gain_in_cam_units
   (&optional __min_gain_in_cam_units)
   (if __min_gain_in_cam_units (setq _min_gain_in_cam_units __min_gain_in_cam_units)) _min_gain_in_cam_units)
  (:max_gain_in_cam_units
   (&optional __max_gain_in_cam_units)
   (if __max_gain_in_cam_units (setq _max_gain_in_cam_units __max_gain_in_cam_units)) _max_gain_in_cam_units)
  (:current_gain_in_cam_units
   (&optional __current_gain_in_cam_units)
   (if __current_gain_in_cam_units (setq _current_gain_in_cam_units __current_gain_in_cam_units)) _current_gain_in_cam_units)
  (:min_gain
   (&optional __min_gain)
   (if __min_gain (setq _min_gain __min_gain)) _min_gain)
  (:max_gain
   (&optional __max_gain)
   (if __max_gain (setq _max_gain __max_gain)) _max_gain)
  (:current_gain
   (&optional __current_gain)
   (if __current_gain (setq _current_gain __current_gain)) _current_gain)
  (:min_gamma
   (&optional __min_gamma)
   (if __min_gamma (setq _min_gamma __min_gamma)) _min_gamma)
  (:max_gamma
   (&optional __max_gamma)
   (if __max_gamma (setq _max_gamma __max_gamma)) _max_gamma)
  (:current_gamma
   (&optional __current_gamma)
   (if __current_gamma (setq _current_gamma __current_gamma)) _current_gamma)
  (:brightness_continuous
   (&optional (__brightness_continuous :null))
   (if (not (eq __brightness_continuous :null)) (setq _brightness_continuous __brightness_continuous)) _brightness_continuous)
  (:gain_auto
   (&optional (__gain_auto :null))
   (if (not (eq __gain_auto :null)) (setq _gain_auto __gain_auto)) _gain_auto)
  (:exposure_auto
   (&optional (__exposure_auto :null))
   (if (not (eq __exposure_auto :null)) (setq _exposure_auto __exposure_auto)) _exposure_auto)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; bool _is_sleeping
    1
    ;; string _device_user_id
    4 (length _device_user_id)
    ;; int32 _min_binning_x
    4
    ;; int32 _max_binning_x
    4
    ;; int32 _current_binning_x
    4
    ;; int32 _min_binning_y
    4
    ;; int32 _max_binning_y
    4
    ;; int32 _current_binning_y
    4
    ;; float32 _max_framerate
    4
    ;; float32 _current_framerate
    4
    ;; float32 _min_exposure
    4
    ;; float32 _max_exposure
    4
    ;; float32 _current_exposure
    4
    ;; float32 _min_gain_in_cam_units
    4
    ;; float32 _max_gain_in_cam_units
    4
    ;; float32 _current_gain_in_cam_units
    4
    ;; float32 _min_gain
    4
    ;; float32 _max_gain
    4
    ;; float32 _current_gain
    4
    ;; float32 _min_gamma
    4
    ;; float32 _max_gamma
    4
    ;; float32 _current_gamma
    4
    ;; bool _brightness_continuous
    1
    ;; bool _gain_auto
    1
    ;; bool _exposure_auto
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_sleeping
       (if _is_sleeping (write-byte -1 s) (write-byte 0 s))
     ;; string _device_user_id
       (write-long (length _device_user_id) s) (princ _device_user_id s)
     ;; int32 _min_binning_x
       (write-long _min_binning_x s)
     ;; int32 _max_binning_x
       (write-long _max_binning_x s)
     ;; int32 _current_binning_x
       (write-long _current_binning_x s)
     ;; int32 _min_binning_y
       (write-long _min_binning_y s)
     ;; int32 _max_binning_y
       (write-long _max_binning_y s)
     ;; int32 _current_binning_y
       (write-long _current_binning_y s)
     ;; float32 _max_framerate
       (sys::poke _max_framerate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_framerate
       (sys::poke _current_framerate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min_exposure
       (sys::poke _min_exposure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_exposure
       (sys::poke _max_exposure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_exposure
       (sys::poke _current_exposure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min_gain_in_cam_units
       (sys::poke _min_gain_in_cam_units (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_gain_in_cam_units
       (sys::poke _max_gain_in_cam_units (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_gain_in_cam_units
       (sys::poke _current_gain_in_cam_units (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min_gain
       (sys::poke _min_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_gain
       (sys::poke _max_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_gain
       (sys::poke _current_gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min_gamma
       (sys::poke _min_gamma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_gamma
       (sys::poke _max_gamma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_gamma
       (sys::poke _current_gamma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _brightness_continuous
       (if _brightness_continuous (write-byte -1 s) (write-byte 0 s))
     ;; bool _gain_auto
       (if _gain_auto (write-byte -1 s) (write-byte 0 s))
     ;; bool _exposure_auto
       (if _exposure_auto (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_sleeping
     (setq _is_sleeping (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _device_user_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _device_user_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _min_binning_x
     (setq _min_binning_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _max_binning_x
     (setq _max_binning_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _current_binning_x
     (setq _current_binning_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _min_binning_y
     (setq _min_binning_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _max_binning_y
     (setq _max_binning_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _current_binning_y
     (setq _current_binning_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _max_framerate
     (setq _max_framerate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_framerate
     (setq _current_framerate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min_exposure
     (setq _min_exposure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_exposure
     (setq _max_exposure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_exposure
     (setq _current_exposure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min_gain_in_cam_units
     (setq _min_gain_in_cam_units (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_gain_in_cam_units
     (setq _max_gain_in_cam_units (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_gain_in_cam_units
     (setq _current_gain_in_cam_units (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min_gain
     (setq _min_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_gain
     (setq _max_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_gain
     (setq _current_gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min_gamma
     (setq _min_gamma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_gamma
     (setq _max_gamma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_gamma
     (setq _current_gamma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _brightness_continuous
     (setq _brightness_continuous (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gain_auto
     (setq _gain_auto (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _exposure_auto
     (setq _exposure_auto (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass camera_control_msgs::GetCamProperties
  :super ros::object
  :slots ())

(setf (get camera_control_msgs::GetCamProperties :md5sum-) "14bd64331efb0f665787f525f453c05d")
(setf (get camera_control_msgs::GetCamProperties :datatype-) "camera_control_msgs/GetCamProperties")
(setf (get camera_control_msgs::GetCamProperties :request) camera_control_msgs::GetCamPropertiesRequest)
(setf (get camera_control_msgs::GetCamProperties :response) camera_control_msgs::GetCamPropertiesResponse)

(defmethod camera_control_msgs::GetCamPropertiesRequest
  (:response () (instance camera_control_msgs::GetCamPropertiesResponse :init)))

(setf (get camera_control_msgs::GetCamPropertiesRequest :md5sum-) "14bd64331efb0f665787f525f453c05d")
(setf (get camera_control_msgs::GetCamPropertiesRequest :datatype-) "camera_control_msgs/GetCamPropertiesRequest")
(setf (get camera_control_msgs::GetCamPropertiesRequest :definition-)
      "# This service returns the current camera properties as well as the limits
---
bool success

bool is_sleeping

string device_user_id

int32 min_binning_x
int32 max_binning_x
int32 current_binning_x

int32 min_binning_y
int32 max_binning_y
int32 current_binning_y

float32 max_framerate
float32 current_framerate

float32 min_exposure
float32 max_exposure
float32 current_exposure

float32 min_gain_in_cam_units
float32 max_gain_in_cam_units
float32 current_gain_in_cam_units

float32 min_gain
float32 max_gain
float32 current_gain

float32 min_gamma
float32 max_gamma
float32 current_gamma

bool brightness_continuous
bool gain_auto
bool exposure_auto

")

(setf (get camera_control_msgs::GetCamPropertiesResponse :md5sum-) "14bd64331efb0f665787f525f453c05d")
(setf (get camera_control_msgs::GetCamPropertiesResponse :datatype-) "camera_control_msgs/GetCamPropertiesResponse")
(setf (get camera_control_msgs::GetCamPropertiesResponse :definition-)
      "# This service returns the current camera properties as well as the limits
---
bool success

bool is_sleeping

string device_user_id

int32 min_binning_x
int32 max_binning_x
int32 current_binning_x

int32 min_binning_y
int32 max_binning_y
int32 current_binning_y

float32 max_framerate
float32 current_framerate

float32 min_exposure
float32 max_exposure
float32 current_exposure

float32 min_gain_in_cam_units
float32 max_gain_in_cam_units
float32 current_gain_in_cam_units

float32 min_gain
float32 max_gain
float32 current_gain

float32 min_gamma
float32 max_gamma
float32 current_gamma

bool brightness_continuous
bool gain_auto
bool exposure_auto

")



(provide :camera_control_msgs/GetCamProperties "14bd64331efb0f665787f525f453c05d")


