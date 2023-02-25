; Auto-generated. Do not edit!


(cl:in-package pid_control-msg)


;//! \htmlinclude set_point.msg.html

(cl:defclass <set_point> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass set_point (<set_point>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <set_point>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'set_point)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pid_control-msg:<set_point> is deprecated: use pid_control-msg:set_point instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <set_point>) ostream)
  "Serializes a message object of type '<set_point>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <set_point>) istream)
  "Deserializes a message object of type '<set_point>"
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
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'set_point)))
  "Returns md5sum for a message object of type 'set_point"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<set_point>)))
  "Returns full string definition for message of type '<set_point>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'set_point)))
  "Returns full string definition for message of type 'set_point"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <set_point>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <set_point>))
  "Converts a ROS message object to a list"
  (cl:list 'set_point
))
