
(cl:in-package :asdf)

(defsystem "pid_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "motor_input" :depends-on ("_package_motor_input"))
    (:file "_package_motor_input" :depends-on ("_package"))
    (:file "motor_output" :depends-on ("_package_motor_output"))
    (:file "_package_motor_output" :depends-on ("_package"))
    (:file "set_point" :depends-on ("_package_set_point"))
    (:file "_package_set_point" :depends-on ("_package"))
  ))