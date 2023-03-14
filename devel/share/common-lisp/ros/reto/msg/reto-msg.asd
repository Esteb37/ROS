
(cl:in-package :asdf)

(defsystem "reto-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "set_point" :depends-on ("_package_set_point"))
    (:file "_package_set_point" :depends-on ("_package"))
  ))