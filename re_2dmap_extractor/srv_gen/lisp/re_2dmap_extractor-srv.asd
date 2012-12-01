
(cl:in-package :asdf)

(defsystem "re_2dmap_extractor-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :re_msgs-msg
)
  :components ((:file "_package")
    (:file "RequestLocMap" :depends-on ("_package_RequestLocMap"))
    (:file "_package_RequestLocMap" :depends-on ("_package"))
    (:file "RequestNavMap" :depends-on ("_package_RequestNavMap"))
    (:file "_package_RequestNavMap" :depends-on ("_package"))
  ))