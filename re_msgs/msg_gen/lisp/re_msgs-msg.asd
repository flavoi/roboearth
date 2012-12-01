
(cl:in-package :asdf)

(defsystem "re_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Pose2DStamped" :depends-on ("_package_Pose2DStamped"))
    (:file "_package_Pose2DStamped" :depends-on ("_package"))
    (:file "Pixel" :depends-on ("_package_Pixel"))
    (:file "_package_Pixel" :depends-on ("_package"))
    (:file "SeenObject" :depends-on ("_package_SeenObject"))
    (:file "_package_SeenObject" :depends-on ("_package"))
    (:file "File" :depends-on ("_package_File"))
    (:file "_package_File" :depends-on ("_package"))
    (:file "StringArray" :depends-on ("_package_StringArray"))
    (:file "_package_StringArray" :depends-on ("_package"))
    (:file "SeenObjectArray" :depends-on ("_package_SeenObjectArray"))
    (:file "_package_SeenObjectArray" :depends-on ("_package"))
    (:file "RosFile" :depends-on ("_package_RosFile"))
    (:file "_package_RosFile" :depends-on ("_package"))
    (:file "DetectedObject" :depends-on ("_package_DetectedObject"))
    (:file "_package_DetectedObject" :depends-on ("_package"))
  ))