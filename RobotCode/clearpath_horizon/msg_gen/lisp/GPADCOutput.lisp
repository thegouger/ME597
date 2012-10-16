; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude GPADCOutput.msg.html

(cl:defclass <GPADCOutput> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GPADCOutput (<GPADCOutput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPADCOutput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPADCOutput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<GPADCOutput> is deprecated: use clearpath_horizon-msg:GPADCOutput instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPADCOutput>) ostream)
  "Serializes a message object of type '<GPADCOutput>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPADCOutput>) istream)
  "Deserializes a message object of type '<GPADCOutput>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPADCOutput>)))
  "Returns string type for a message object of type '<GPADCOutput>"
  "clearpath_horizon/GPADCOutput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPADCOutput)))
  "Returns string type for a message object of type 'GPADCOutput"
  "clearpath_horizon/GPADCOutput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPADCOutput>)))
  "Returns md5sum for a message object of type '<GPADCOutput>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPADCOutput)))
  "Returns md5sum for a message object of type 'GPADCOutput"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPADCOutput>)))
  "Returns full string definition for message of type '<GPADCOutput>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPADCOutput)))
  "Returns full string definition for message of type 'GPADCOutput"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPADCOutput>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPADCOutput>))
  "Converts a ROS message object to a list"
  (cl:list 'GPADCOutput
))
