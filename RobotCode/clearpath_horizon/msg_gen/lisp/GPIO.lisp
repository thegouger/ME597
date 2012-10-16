; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude GPIO.msg.html

(cl:defclass <GPIO> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GPIO (<GPIO>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPIO>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPIO)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<GPIO> is deprecated: use clearpath_horizon-msg:GPIO instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPIO>) ostream)
  "Serializes a message object of type '<GPIO>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPIO>) istream)
  "Deserializes a message object of type '<GPIO>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPIO>)))
  "Returns string type for a message object of type '<GPIO>"
  "clearpath_horizon/GPIO")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPIO)))
  "Returns string type for a message object of type 'GPIO"
  "clearpath_horizon/GPIO")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPIO>)))
  "Returns md5sum for a message object of type '<GPIO>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPIO)))
  "Returns md5sum for a message object of type 'GPIO"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPIO>)))
  "Returns full string definition for message of type '<GPIO>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPIO)))
  "Returns full string definition for message of type 'GPIO"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPIO>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPIO>))
  "Converts a ROS message object to a list"
  (cl:list 'GPIO
))
