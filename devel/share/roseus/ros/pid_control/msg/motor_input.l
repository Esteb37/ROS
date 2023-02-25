;; Auto-generated. Do not edit!


(when (boundp 'pid_control::motor_input)
  (if (not (find-package "PID_CONTROL"))
    (make-package "PID_CONTROL"))
  (shadow 'motor_input (find-package "PID_CONTROL")))
(unless (find-package "PID_CONTROL::MOTOR_INPUT")
  (make-package "PID_CONTROL::MOTOR_INPUT"))

(in-package "ROS")
;;//! \htmlinclude motor_input.msg.html


(defclass pid_control::motor_input
  :super ros::object
  :slots (_input _time ))

(defmethod pid_control::motor_input
  (:init
   (&key
    ((:input __input) 0.0)
    ((:time __time) 0.0)
    )
   (send-super :init)
   (setq _input (float __input))
   (setq _time (float __time))
   self)
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; float32 _input
    4
    ;; float64 _time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _input
       (sys::poke _input (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _input
     (setq _input (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get pid_control::motor_input :md5sum-) "68938b46224a92992b7cd8fd6233cc91")
(setf (get pid_control::motor_input :datatype-) "pid_control/motor_input")
(setf (get pid_control::motor_input :definition-)
      "float32 input
float64 time

")



(provide :pid_control/motor_input "68938b46224a92992b7cd8fd6233cc91")


