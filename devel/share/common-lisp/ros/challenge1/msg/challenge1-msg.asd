
(cl:in-package :asdf)

(defsystem "challenge1-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "script_select" :depends-on ("_package_script_select"))
    (:file "_package_script_select" :depends-on ("_package"))
  ))