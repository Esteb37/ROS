;; Auto-generated. Do not edit!


(when (boundp 'reto::set_point)
  (if (not (find-package "RETO"))
    (make-package "RETO"))
  (shadow 'set_point (find-package "RETO")))
(unless (find-package "RETO::SET_POINT")
  (make-package "RETO::SET_POINT"))

(in-package "ROS")
;;//! \htmlinclude set_point.msg.html


(defclass reto::set_point
  :super ros::object
  :slots (_value _mode ))

(defmethod reto::set_point
  (:init
   (&key
    ((:value __value) 0.0)
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _value (float __value))
   (setq _mode (round __mode))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; float32 _value
    4
    ;; int32 _mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _mode
       (write-long _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get reto::set_point :md5sum-) "13ab31516f5e73a712f39652b01f4fe5")
(setf (get reto::set_point :datatype-) "reto/set_point")
(setf (get reto::set_point :definition-)
      "float32 value
int32 mode

")



(provide :reto/set_point "13ab31516f5e73a712f39652b01f4fe5")


