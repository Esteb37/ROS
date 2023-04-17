
(cl:in-package :asdf)

(defsystem "open_path_follower-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "script_select" :depends-on ("_package_script_select"))
    (:file "_package_script_select" :depends-on ("_package"))
  ))