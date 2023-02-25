; Auto-generated. Do not edit!


(cl:in-package pid_control-msg)


;//! \htmlinclude motor_input.msg.html

(cl:defclass <motor_input> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type cl:float
    :initform 0.0)
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0))
)

(cl:defclass motor_input (<motor_input>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_input>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_input)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pid_control-msg:<motor_input> is deprecated: use pid_control-msg:motor_input instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <motor_input>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pid_control-msg:input-val is deprecated.  Use pid_control-msg:input instead.")
  (input m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <motor_input>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pid_control-msg:time-val is deprecated.  Use pid_control-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_input>) ostream)
  "Serializes a message object of type '<motor_input>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'input))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_input>) istream)
  "Deserializes a message object of type '<motor_input>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_input>)))
  "Returns string type for a message object of type '<motor_input>"
  "pid_control/motor_input")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_input)))
  "Returns string type for a message object of type 'motor_input"
  "pid_control/motor_input")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_input>)))
  "Returns md5sum for a message object of type '<motor_input>"
  "68938b46224a92992b7cd8fd6233cc91")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_input)))
  "Returns md5sum for a message object of type 'motor_input"
  "68938b46224a92992b7cd8fd6233cc91")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_input>)))
  "Returns full string definition for message of type '<motor_input>"
  (cl:format cl:nil "float32 input~%float64 time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_input)))
  "Returns full string definition for message of type 'motor_input"
  (cl:format cl:nil "float32 input~%float64 time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_input>))
  (cl:+ 0
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_input>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_input
    (cl:cons ':input (input msg))
    (cl:cons ':time (time msg))
))
