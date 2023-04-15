; Auto-generated. Do not edit!


(cl:in-package challenge1-msg)


;//! \htmlinclude script_select.msg.html

(cl:defclass <script_select> (roslisp-msg-protocol:ros-message)
  ((script_select
    :reader script_select
    :initarg :script_select
    :type cl:string
    :initform "")
   (type_select
    :reader type_select
    :initarg :type_select
    :type cl:string
    :initform "")
   (turn_velocity
    :reader turn_velocity
    :initarg :turn_velocity
    :type cl:float
    :initform 0.0)
   (move_velocity
    :reader move_velocity
    :initarg :move_velocity
    :type cl:float
    :initform 0.0)
   (total_time
    :reader total_time
    :initarg :total_time
    :type cl:float
    :initform 0.0)
   (square_length
    :reader square_length
    :initarg :square_length
    :type cl:float
    :initform 0.0))
)

(cl:defclass script_select (<script_select>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <script_select>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'script_select)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name challenge1-msg:<script_select> is deprecated: use challenge1-msg:script_select instead.")))

(cl:ensure-generic-function 'script_select-val :lambda-list '(m))
(cl:defmethod script_select-val ((m <script_select>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader challenge1-msg:script_select-val is deprecated.  Use challenge1-msg:script_select instead.")
  (script_select m))

(cl:ensure-generic-function 'type_select-val :lambda-list '(m))
(cl:defmethod type_select-val ((m <script_select>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader challenge1-msg:type_select-val is deprecated.  Use challenge1-msg:type_select instead.")
  (type_select m))

(cl:ensure-generic-function 'turn_velocity-val :lambda-list '(m))
(cl:defmethod turn_velocity-val ((m <script_select>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader challenge1-msg:turn_velocity-val is deprecated.  Use challenge1-msg:turn_velocity instead.")
  (turn_velocity m))

(cl:ensure-generic-function 'move_velocity-val :lambda-list '(m))
(cl:defmethod move_velocity-val ((m <script_select>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader challenge1-msg:move_velocity-val is deprecated.  Use challenge1-msg:move_velocity instead.")
  (move_velocity m))

(cl:ensure-generic-function 'total_time-val :lambda-list '(m))
(cl:defmethod total_time-val ((m <script_select>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader challenge1-msg:total_time-val is deprecated.  Use challenge1-msg:total_time instead.")
  (total_time m))

(cl:ensure-generic-function 'square_length-val :lambda-list '(m))
(cl:defmethod square_length-val ((m <script_select>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader challenge1-msg:square_length-val is deprecated.  Use challenge1-msg:square_length instead.")
  (square_length m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <script_select>) ostream)
  "Serializes a message object of type '<script_select>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'script_select))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'script_select))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type_select))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type_select))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'turn_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'move_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'total_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'square_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <script_select>) istream)
  "Deserializes a message object of type '<script_select>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'script_select) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'script_select) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type_select) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type_select) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'turn_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'move_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'square_length) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<script_select>)))
  "Returns string type for a message object of type '<script_select>"
  "challenge1/script_select")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'script_select)))
  "Returns string type for a message object of type 'script_select"
  "challenge1/script_select")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<script_select>)))
  "Returns md5sum for a message object of type '<script_select>"
  "dde5cb34cae0c7a4f86426afcd558fdc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'script_select)))
  "Returns md5sum for a message object of type 'script_select"
  "dde5cb34cae0c7a4f86426afcd558fdc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<script_select>)))
  "Returns full string definition for message of type '<script_select>"
  (cl:format cl:nil "string script_select~%string type_select~%float32 turn_velocity~%float32 move_velocity~%float32 total_time~%float32 square_length~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'script_select)))
  "Returns full string definition for message of type 'script_select"
  (cl:format cl:nil "string script_select~%string type_select~%float32 turn_velocity~%float32 move_velocity~%float32 total_time~%float32 square_length~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <script_select>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'script_select))
     4 (cl:length (cl:slot-value msg 'type_select))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <script_select>))
  "Converts a ROS message object to a list"
  (cl:list 'script_select
    (cl:cons ':script_select (script_select msg))
    (cl:cons ':type_select (type_select msg))
    (cl:cons ':turn_velocity (turn_velocity msg))
    (cl:cons ':move_velocity (move_velocity msg))
    (cl:cons ':total_time (total_time msg))
    (cl:cons ':square_length (square_length msg))
))
