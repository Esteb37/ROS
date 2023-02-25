;; Auto-generated. Do not edit!


(when (boundp 'pid_control::motor_output)
  (if (not (find-package "PID_CONTROL"))
    (make-package "PID_CONTROL"))
  (shadow 'motor_output (find-package "PID_CONTROL")))
(unless (find-package "PID_CONTROL::MOTOR_OUTPUT")
  (make-package "PID_CONTROL::MOTOR_OUTPUT"))

(in-package "ROS")
;;//! \htmlinclude motor_output.msg.html


(defclass pid_control::motor_output
  :super ros::object
  :slots (_output _time _status ))

(defmethod pid_control::motor_output
  (:init
   (&key
    ((:output __output) 0.0)
    ((:time __time) 0.0)
    ((:status __status) "")
    )
   (send-super :init)
   (setq _output (float __output))
   (setq _time (float __time))
   (setq _status (string __status))
   self)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; float32 _output
    4
    ;; float64 _time
    8
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _output
       (sys::poke _output (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _output
     (setq _output (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pid_control::motor_output :md5sum-) "6f371a723b35f6884908910ed6144b2d")
(setf (get pid_control::motor_output :datatype-) "pid_control/motor_output")
(setf (get pid_control::motor_output :definition-)
      "float32 output
float64 time
string status
")



(provide :pid_control/motor_output "6f371a723b35f6884908910ed6144b2d")


