; Auto-generated. Do not edit!


(cl:in-package indoor_pos-msg)


;//! \htmlinclude ips_msg.msg.html

(cl:defclass <ips_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (X
    :reader X
    :initarg :X
    :type cl:float
    :initform 0.0)
   (Y
    :reader Y
    :initarg :Y
    :type cl:float
    :initform 0.0)
   (Yaw
    :reader Yaw
    :initarg :Yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass ips_msg (<ips_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ips_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ips_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name indoor_pos-msg:<ips_msg> is deprecated: use indoor_pos-msg:ips_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ips_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader indoor_pos-msg:header-val is deprecated.  Use indoor_pos-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'X-val :lambda-list '(m))
(cl:defmethod X-val ((m <ips_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader indoor_pos-msg:X-val is deprecated.  Use indoor_pos-msg:X instead.")
  (X m))

(cl:ensure-generic-function 'Y-val :lambda-list '(m))
(cl:defmethod Y-val ((m <ips_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader indoor_pos-msg:Y-val is deprecated.  Use indoor_pos-msg:Y instead.")
  (Y m))

(cl:ensure-generic-function 'Yaw-val :lambda-list '(m))
(cl:defmethod Yaw-val ((m <ips_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader indoor_pos-msg:Yaw-val is deprecated.  Use indoor_pos-msg:Yaw instead.")
  (Yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ips_msg>) ostream)
  "Serializes a message object of type '<ips_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'X))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ips_msg>) istream)
  "Deserializes a message object of type '<ips_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'X) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ips_msg>)))
  "Returns string type for a message object of type '<ips_msg>"
  "indoor_pos/ips_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ips_msg)))
  "Returns string type for a message object of type 'ips_msg"
  "indoor_pos/ips_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ips_msg>)))
  "Returns md5sum for a message object of type '<ips_msg>"
  "bbe3bb11305e11f0c913565ed1efb960")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ips_msg)))
  "Returns md5sum for a message object of type 'ips_msg"
  "bbe3bb11305e11f0c913565ed1efb960")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ips_msg>)))
  "Returns full string definition for message of type '<ips_msg>"
  (cl:format cl:nil "Header header~%~%float32 X~%float32 Y~%float32 Yaw~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ips_msg)))
  "Returns full string definition for message of type 'ips_msg"
  (cl:format cl:nil "Header header~%~%float32 X~%float32 Y~%float32 Yaw~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ips_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ips_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'ips_msg
    (cl:cons ':header (header msg))
    (cl:cons ':X (X msg))
    (cl:cons ':Y (Y msg))
    (cl:cons ':Yaw (Yaw msg))
))
