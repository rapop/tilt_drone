
(cl:in-package :asdf)

(defsystem "sensor_fusion_comm-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "InitHeight" :depends-on ("_package_InitHeight"))
    (:file "_package_InitHeight" :depends-on ("_package"))
    (:file "InitScale" :depends-on ("_package_InitScale"))
    (:file "_package_InitScale" :depends-on ("_package"))
  ))