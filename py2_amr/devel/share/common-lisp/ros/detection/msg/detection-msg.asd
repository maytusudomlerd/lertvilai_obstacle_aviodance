
(cl:in-package :asdf)

(defsystem "detection-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "obstacle_bound" :depends-on ("_package_obstacle_bound"))
    (:file "_package_obstacle_bound" :depends-on ("_package"))
  ))