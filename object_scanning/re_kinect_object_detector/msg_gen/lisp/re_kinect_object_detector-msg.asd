
(cl:in-package :asdf)

(defsystem "re_kinect_object_detector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :re_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "DetectionResult" :depends-on ("_package_DetectionResult"))
    (:file "_package_DetectionResult" :depends-on ("_package"))
  ))