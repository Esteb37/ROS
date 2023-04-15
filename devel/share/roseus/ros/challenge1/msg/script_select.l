;; Auto-generated. Do not edit!


(when (boundp 'challenge1::script_select)
  (if (not (find-package "CHALLENGE1"))
    (make-package "CHALLENGE1"))
  (shadow 'script_select (find-package "CHALLENGE1")))
(unless (find-package "CHALLENGE1::SCRIPT_SELECT")
  (make-package "CHALLENGE1::SCRIPT_SELECT"))

(in-package "ROS")
;;//! \htmlinclude script_select.msg.html


(defclass challenge1::script_select
  :super ros::object
  :slots (_script_select _type_select _turn_velocity _move_velocity _total_time _square_length ))

(defmethod challenge1::script_select
  (:init
   (&key
    ((:script_select __script_select) "")
    ((:type_select __type_select) "")
    ((:turn_velocity __turn_velocity) 0.0)
    ((:move_velocity __move_velocity) 0.0)
    ((:total_time __total_time) 0.0)
    ((:square_length __square_length) 0.0)
    )
   (send-super :init)
   (setq _script_select (string __script_select))
   (setq _type_select (string __type_select))
   (setq _turn_velocity (float __turn_velocity))
   (setq _move_velocity (float __move_velocity))
   (setq _total_time (float __total_time))
   (setq _square_length (float __square_length))
   self)
  (:script_select
   (&optional __script_select)
   (if __script_select (setq _script_select __script_select)) _script_select)
  (:type_select
   (&optional __type_select)
   (if __type_select (setq _type_select __type_select)) _type_select)
  (:turn_velocity
   (&optional __turn_velocity)
   (if __turn_velocity (setq _turn_velocity __turn_velocity)) _turn_velocity)
  (:move_velocity
   (&optional __move_velocity)
   (if __move_velocity (setq _move_velocity __move_velocity)) _move_velocity)
  (:total_time
   (&optional __total_time)
   (if __total_time (setq _total_time __total_time)) _total_time)
  (:square_length
   (&optional __square_length)
   (if __square_length (setq _square_length __square_length)) _square_length)
  (:serialization-length
   ()
   (+
    ;; string _script_select
    4 (length _script_select)
    ;; string _type_select
    4 (length _type_select)
    ;; float32 _turn_velocity
    4
    ;; float32 _move_velocity
    4
    ;; float32 _total_time
    4
    ;; float32 _square_length
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _script_select
       (write-long (length _script_select) s) (princ _script_select s)
     ;; string _type_select
       (write-long (length _type_select) s) (princ _type_select s)
     ;; float32 _turn_velocity
       (sys::poke _turn_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _move_velocity
       (sys::poke _move_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _total_time
       (sys::poke _total_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _square_length
       (sys::poke _square_length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _script_select
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _script_select (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _type_select
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type_select (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _turn_velocity
     (setq _turn_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _move_velocity
     (setq _move_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _total_time
     (setq _total_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _square_length
     (setq _square_length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get challenge1::script_select :md5sum-) "dde5cb34cae0c7a4f86426afcd558fdc")
(setf (get challenge1::script_select :datatype-) "challenge1/script_select")
(setf (get challenge1::script_select :definition-)
      "string script_select
string type_select
float32 turn_velocity
float32 move_velocity
float32 total_time
float32 square_length

")



(provide :challenge1/script_select "dde5cb34cae0c7a4f86426afcd558fdc")


