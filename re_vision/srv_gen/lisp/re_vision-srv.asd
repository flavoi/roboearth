
(cl:in-package :asdf)

(defsystem "re_vision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :re_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "SearchFor" :depends-on ("_package_SearchFor"))
    (:file "_package_SearchFor" :depends-on ("_package"))
  ))