; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude RawEncoders.msg.html

(cl:defclass <RawEncoders> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ticks
    :reader ticks
    :initarg :ticks
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass RawEncoders (<RawEncoders>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RawEncoders>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RawEncoders)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<RawEncoders> is deprecated: use clearpath_horizon-msg:RawEncoders instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RawEncoders>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:header-val is deprecated.  Use clearpath_horizon-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ticks-val :lambda-list '(m))
(cl:defmethod ticks-val ((m <RawEncoders>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:ticks-val is deprecated.  Use clearpath_horizon-msg:ticks instead.")
  (ticks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RawEncoders>) ostream)
  "Serializes a message object of type '<RawEncoders>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ticks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'ticks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RawEncoders>) istream)
  "Deserializes a message object of type '<RawEncoders>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ticks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ticks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RawEncoders>)))
  "Returns string type for a message object of type '<RawEncoders>"
  "clearpath_horizon/RawEncoders")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RawEncoders)))
  "Returns string type for a message object of type 'RawEncoders"
  "clearpath_horizon/RawEncoders")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RawEncoders>)))
  "Returns md5sum for a message object of type '<RawEncoders>"
  "d105c5e4b407d4a68215fce86175fe75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RawEncoders)))
  "Returns md5sum for a message object of type 'RawEncoders"
  "d105c5e4b407d4a68215fce86175fe75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RawEncoders>)))
  "Returns full string definition for message of type '<RawEncoders>"
  (cl:format cl:nil "Header header~%int32[] ticks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RawEncoders)))
  "Returns full string definition for message of type 'RawEncoders"
  (cl:format cl:nil "Header header~%int32[] ticks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RawEncoders>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ticks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RawEncoders>))
  "Converts a ROS message object to a list"
  (cl:list 'RawEncoders
    (cl:cons ':header (header msg))
    (cl:cons ':ticks (ticks msg))
))
