
(cl:in-package :asdf)

(defsystem "depthai_ros_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
               :vision_msgs-msg
               :visualization_msgs-msg
)
  :components ((:file "_package")
    (:file "AutoFocusCtrl" :depends-on ("_package_AutoFocusCtrl"))
    (:file "_package_AutoFocusCtrl" :depends-on ("_package"))
    (:file "HandLandmark" :depends-on ("_package_HandLandmark"))
    (:file "_package_HandLandmark" :depends-on ("_package"))
    (:file "HandLandmarkArray" :depends-on ("_package_HandLandmarkArray"))
    (:file "_package_HandLandmarkArray" :depends-on ("_package"))
    (:file "ImageMarker" :depends-on ("_package_ImageMarker"))
    (:file "_package_ImageMarker" :depends-on ("_package"))
    (:file "ImageMarkerArray" :depends-on ("_package_ImageMarkerArray"))
    (:file "_package_ImageMarkerArray" :depends-on ("_package"))
    (:file "ImuWithMagneticField" :depends-on ("_package_ImuWithMagneticField"))
    (:file "_package_ImuWithMagneticField" :depends-on ("_package"))
    (:file "SpatialDetection" :depends-on ("_package_SpatialDetection"))
    (:file "_package_SpatialDetection" :depends-on ("_package"))
    (:file "SpatialDetectionArray" :depends-on ("_package_SpatialDetectionArray"))
    (:file "_package_SpatialDetectionArray" :depends-on ("_package"))
    (:file "TrackedFeature" :depends-on ("_package_TrackedFeature"))
    (:file "_package_TrackedFeature" :depends-on ("_package"))
    (:file "TrackedFeatures" :depends-on ("_package_TrackedFeatures"))
    (:file "_package_TrackedFeatures" :depends-on ("_package"))
  ))