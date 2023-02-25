; Auto-generated. Do not edit!


(cl:in-package pid_control-msg)


;//! \htmlinclude set_point.msg.html

(cl:defclass <set_point> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0)
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0))
)

(cl:defclass set_point (<set_point>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_point>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_point)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pid_control-msg:<set_point> is deprecated: use pid_control-msg:set_point instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <set_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pid_control-msg:value-val is deprecated.  Use pid_control-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <set_point>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pid_control-msg:time-val is deprecated.  Use pid_control-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_point>) ostream)
  "Serializes a message object of type '<set_point>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_point>) istream)
  "Deserializes a message object of type '<set_point>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<set_point>)))
  "Returns string type for a message object of type '<set_point>"
  "pid_control/set_point")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'set_point)))
  "Returns string type for a message object of type 'set_point"
  "pid_control/set_point")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<set_point>)))
  "Returns md5sum for a message object of type '<set_point>"
  "01346652ed5c09b39a6e088152e53548")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_point)))
  "Returns md5sum for a message object of type 'set_point"
  "01346652ed5c09b39a6e088152e53548")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_point>)))
  "Returns full string definition for message of type '<set_point>"
  (cl:format cl:nil "float32 value~%float32 time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_point)))
  "Returns full string definition for message of type 'set_point"
  (cl:format cl:nil "float32 value~%float32 time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_point>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_point>))
  "Converts a ROS message object to a list"
  (cl:list 'set_point
    (cl:cons ':value (value msg))
    (cl:cons ':time (time msg))
))
