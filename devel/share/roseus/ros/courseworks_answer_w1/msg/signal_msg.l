;; Auto-generated. Do not edit!


(when (boundp 'courseworks_answer_w1::signal_msg)
  (if (not (find-package "COURSEWORKS_ANSWER_W1"))
    (make-package "COURSEWORKS_ANSWER_W1"))
  (shadow 'signal_msg (find-package "COURSEWORKS_ANSWER_W1")))
(unless (find-package "COURSEWORKS_ANSWER_W1::SIGNAL_MSG")
  (make-package "COURSEWORKS_ANSWER_W1::SIGNAL_MSG"))

(in-package "ROS")
;;//! \htmlinclude signal_msg.msg.html


(defclass courseworks_answer_w1::signal_msg
  :super ros::object
  :slots (_time_x _signal_y ))

(defmethod courseworks_answer_w1::signal_msg
  (:init
   (&key
    ((:time_x __time_x) 0.0)
    ((:signal_y __signal_y) 0.0)
    )
   (send-super :init)
   (setq _time_x (float __time_x))
   (setq _signal_y (float __signal_y))
   self)
  (:time_x
   (&optional __time_x)
   (if __time_x (setq _time_x __time_x)) _time_x)
  (:signal_y
   (&optional __signal_y)
   (if __signal_y (setq _signal_y __signal_y)) _signal_y)
  (:serialization-length
   ()
   (+
    ;; float32 _time_x
    4
    ;; float32 _signal_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _time_x
       (sys::poke _time_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _signal_y
       (sys::poke _signal_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _time_x
     (setq _time_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _signal_y
     (setq _signal_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get courseworks_answer_w1::signal_msg :md5sum-) "7a11f0834f5ad70ec78337929f76544a")
(setf (get courseworks_answer_w1::signal_msg :datatype-) "courseworks_answer_w1/signal_msg")
(setf (get courseworks_answer_w1::signal_msg :definition-)
      "float32 time_x
float32 signal_y

")



(provide :courseworks_answer_w1/signal_msg "7a11f0834f5ad70ec78337929f76544a")


