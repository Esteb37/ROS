; Auto-generated. Do not edit!


(cl:in-package pid_control-msg)


;//! \htmlinclude motor_output.msg.html

(cl:defclass <motor_output> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:float
    :initform 0.0)
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0)
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass motor_output (<motor_output>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_output>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_output)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pid_control-msg:<motor_output> is deprecated: use pid_control-msg:motor_output instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <motor_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pid_control-msg:output-val is deprecated.  Use pid_control-msg:output instead.")
  (output m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <motor_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pid_control-msg:time-val is deprecated.  Use pid_control-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <motor_output>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pid_control-msg:status-val is deprecated.  Use pid_control-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_output>) ostream)
  "Serializes a message object of type '<motor_output>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'output))))
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_output>) istream)
  "Deserializes a message object of type '<motor_output>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'output) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_output>)))
  "Returns string type for a message object of type '<motor_output>"
  "pid_control/motor_output")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_output)))
  "Returns string type for a message object of type 'motor_output"
  "pid_control/motor_output")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_output>)))
  "Returns md5sum for a message object of type '<motor_output>"
  "6f371a723b35f6884908910ed6144b2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_output)))
  "Returns md5sum for a message object of type 'motor_output"
  "6f371a723b35f6884908910ed6144b2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_output>)))
  "Returns full string definition for message of type '<motor_output>"
  (cl:format cl:nil "float32 output~%float64 time~%string status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_output)))
  "Returns full string definition for message of type 'motor_output"
  (cl:format cl:nil "float32 output~%float64 time~%string status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_output>))
  (cl:+ 0
     4
     8
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_output>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_output
    (cl:cons ':output (output msg))
    (cl:cons ':time (time msg))
    (cl:cons ':status (status msg))
))
