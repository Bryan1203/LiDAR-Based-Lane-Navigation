
(cl:in-package :asdf)

(defsystem "depthai_ros_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "NormalizedImageCrop" :depends-on ("_package_NormalizedImageCrop"))
    (:file "_package_NormalizedImageCrop" :depends-on ("_package"))
    (:file "TriggerNamed" :depends-on ("_package_TriggerNamed"))
    (:file "_package_TriggerNamed" :depends-on ("_package"))
  ))