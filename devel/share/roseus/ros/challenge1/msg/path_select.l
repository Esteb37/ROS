;; Auto-generated. Do not edit!


(when (boundp 'challenge1::path_select)
  (if (not (find-package "CHALLENGE1"))
    (make-package "CHALLENGE1"))
  (shadow 'path_select (find-package "CHALLENGE1")))
(unless (find-package "CHALLENGE1::PATH_SELECT")
  (make-package "CHALLENGE1::PATH_SELECT"))

(in-package "ROS")
;;//! \htmlinclude path_select.msg.html


(defclass challenge1::path_select
  :super ros::object
  :slots (_script_select _type_select _vel_or_time _square_length ))

(defmethod challenge1::path_select
  (:init
   (&key
    ((:script_select __script_select) "")
    ((:type_select __type_select) "")
    ((:vel_or_time __vel_or_time) 0.0)
    ((:square_length __square_length) 0.0)
    )
   (send-super :init)
   (setq _script_select (string __script_select))
   (setq _type_select (string __type_select))
   (setq _vel_or_time (float __vel_or_time))
   (setq _square_length (float __square_length))
   self)
  (:script_select
   (&optional __script_select)
   (if __script_select (setq _script_select __script_select)) _script_select)
  (:type_select
   (&optional __type_select)
   (if __type_select (setq _type_select __type_select)) _type_select)
  (:vel_or_time
   (&optional __vel_or_time)
   (if __vel_or_time (setq _vel_or_time __vel_or_time)) _vel_or_time)
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
    ;; float32 _vel_or_time
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
     ;; float32 _vel_or_time
       (sys::poke _vel_or_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _vel_or_time
     (setq _vel_or_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _square_length
     (setq _square_length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get challenge1::path_select :md5sum-) "018261c4027250dddc46523e31e3866b")
(setf (get challenge1::path_select :datatype-) "challenge1/path_select")
(setf (get challenge1::path_select :definition-)
      "string script_select
string type_select
float32 vel_or_time
float32 square_length
")



(provide :challenge1/path_select "018261c4027250dddc46523e31e3866b")


