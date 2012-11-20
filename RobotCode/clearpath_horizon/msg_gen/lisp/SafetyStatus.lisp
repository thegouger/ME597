; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude SafetyStatus.msg.html

(cl:defclass <SafetyStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (flags
    :reader flags
    :initarg :flags
    :type cl:fixnum
    :initform 0)
   (estop
    :reader estop
    :initarg :estop
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SafetyStatus (<SafetyStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<SafetyStatus> is deprecated: use clearpath_horizon-msg:SafetyStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SafetyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:header-val is deprecated.  Use clearpath_horizon-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <SafetyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:flags-val is deprecated.  Use clearpath_horizon-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'estop-val :lambda-list '(m))
(cl:defmethod estop-val ((m <SafetyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:estop-val is deprecated.  Use clearpath_horizon-msg:estop instead.")
  (estop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyStatus>) ostream)
  "Serializes a message object of type '<SafetyStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'estop) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyStatus>) istream)
  "Deserializes a message object of type '<SafetyStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'estop) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyStatus>)))
  "Returns string type for a message object of type '<SafetyStatus>"
  "clearpath_horizon/SafetyStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyStatus)))
  "Returns string type for a message object of type 'SafetyStatus"
  "clearpath_horizon/SafetyStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyStatus>)))
  "Returns md5sum for a message object of type '<SafetyStatus>"
  "cf78d6042b92d64ebda55641e06d66fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyStatus)))
  "Returns md5sum for a message object of type 'SafetyStatus"
  "cf78d6042b92d64ebda55641e06d66fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyStatus>)))
  "Returns full string definition for message of type '<SafetyStatus>"
  (cl:format cl:nil "Header header~%uint16 flags~%bool estop~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyStatus)))
  "Returns full string definition for message of type 'SafetyStatus"
  (cl:format cl:nil "Header header~%uint16 flags~%bool estop~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyStatus
    (cl:cons ':header (header msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':estop (estop msg))
))
