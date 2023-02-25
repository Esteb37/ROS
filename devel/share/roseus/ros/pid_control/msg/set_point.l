;; Auto-generated. Do not edit!


(when (boundp 'pid_control::set_point)
  (if (not (find-package "PID_CONTROL"))
    (make-package "PID_CONTROL"))
  (shadow 'set_point (find-package "PID_CONTROL")))
(unless (find-package "PID_CONTROL::SET_POINT")
  (make-package "PID_CONTROL::SET_POINT"))

(in-package "ROS")
;;//! \htmlinclude set_point.msg.html


(defclass pid_control::set_point
  :super ros::object
  :slots ())

(defmethod pid_control::set_point
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get pid_control::set_point :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get pid_control::set_point :datatype-) "pid_control/set_point")
(setf (get pid_control::set_point :definition-)
      "
")



(provide :pid_control/set_point "d41d8cd98f00b204e9800998ecf8427e")


