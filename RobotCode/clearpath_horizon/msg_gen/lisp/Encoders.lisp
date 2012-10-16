; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude Encoders.msg.html

(cl:defclass <Encoders> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (encoders
    :reader encoders
    :initarg :encoders
    :type (cl:vector clearpath_horizon-msg:Encoder)
   :initform (cl:make-array 0 :element-type 'clearpath_horizon-msg:Encoder :initial-element (cl:make-instance 'clearpath_horizon-msg:Encoder))))
)

(cl:defclass Encoders (<Encoders>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Encoders>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Encoders)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<Encoders> is deprecated: use clearpath_horizon-msg:Encoders instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Encoders>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:header-val is deprecated.  Use clearpath_horizon-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'encoders-val :lambda-list '(m))
(cl:defmethod encoders-val ((m <Encoders>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:encoders-val is deprecated.  Use clearpath_horizon-msg:encoders instead.")
  (encoders m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Encoders>) ostream)
  "Serializes a message object of type '<Encoders>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'encoders))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'encoders))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Encoders>) istream)
  "Deserializes a message object of type '<Encoders>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'encoders) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'encoders)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'clearpath_horizon-msg:Encoder))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Encoders>)))
  "Returns string type for a message object of type '<Encoders>"
  "clearpath_horizon/Encoders")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Encoders)))
  "Returns string type for a message object of type 'Encoders"
  "clearpath_horizon/Encoders")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Encoders>)))
  "Returns md5sum for a message object of type '<Encoders>"
  "2ea748832c2014369ffabd316d5aad8c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Encoders)))
  "Returns md5sum for a message object of type 'Encoders"
  "2ea748832c2014369ffabd316d5aad8c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Encoders>)))
  "Returns full string definition for message of type '<Encoders>"
  (cl:format cl:nil "Header header~%Encoder[] encoders~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: clearpath_horizon/Encoder~%float64 travel~%float64 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Encoders)))
  "Returns full string definition for message of type 'Encoders"
  (cl:format cl:nil "Header header~%Encoder[] encoders~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: clearpath_horizon/Encoder~%float64 travel~%float64 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Encoders>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'encoders) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Encoders>))
  "Converts a ROS message object to a list"
  (cl:list 'Encoders
    (cl:cons ':header (header msg))
    (cl:cons ':encoders (encoders msg))
))
