; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude PlatformInfo.msg.html

(cl:defclass <PlatformInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (model
    :reader model
    :initarg :model
    :type cl:string
    :initform "")
   (revision
    :reader revision
    :initarg :revision
    :type cl:fixnum
    :initform 0)
   (serial
    :reader serial
    :initarg :serial
    :type cl:integer
    :initform 0))
)

(cl:defclass PlatformInfo (<PlatformInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlatformInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlatformInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<PlatformInfo> is deprecated: use clearpath_horizon-msg:PlatformInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PlatformInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:header-val is deprecated.  Use clearpath_horizon-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'model-val :lambda-list '(m))
(cl:defmethod model-val ((m <PlatformInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:model-val is deprecated.  Use clearpath_horizon-msg:model instead.")
  (model m))

(cl:ensure-generic-function 'revision-val :lambda-list '(m))
(cl:defmethod revision-val ((m <PlatformInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:revision-val is deprecated.  Use clearpath_horizon-msg:revision instead.")
  (revision m))

(cl:ensure-generic-function 'serial-val :lambda-list '(m))
(cl:defmethod serial-val ((m <PlatformInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:serial-val is deprecated.  Use clearpath_horizon-msg:serial instead.")
  (serial m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlatformInfo>) ostream)
  "Serializes a message object of type '<PlatformInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model))
  (cl:let* ((signed (cl:slot-value msg 'revision)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'serial)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'serial)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'serial)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'serial)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlatformInfo>) istream)
  "Deserializes a message object of type '<PlatformInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'revision) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'serial)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'serial)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'serial)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'serial)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlatformInfo>)))
  "Returns string type for a message object of type '<PlatformInfo>"
  "clearpath_horizon/PlatformInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlatformInfo)))
  "Returns string type for a message object of type 'PlatformInfo"
  "clearpath_horizon/PlatformInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlatformInfo>)))
  "Returns md5sum for a message object of type '<PlatformInfo>"
  "ff95c25c6ef78f06bbb7ef85aad5735e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlatformInfo)))
  "Returns md5sum for a message object of type 'PlatformInfo"
  "ff95c25c6ef78f06bbb7ef85aad5735e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlatformInfo>)))
  "Returns full string definition for message of type '<PlatformInfo>"
  (cl:format cl:nil "Header header~%string model~%int8 revision~%uint32 serial~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlatformInfo)))
  "Returns full string definition for message of type 'PlatformInfo"
  (cl:format cl:nil "Header header~%string model~%int8 revision~%uint32 serial~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlatformInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'model))
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlatformInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'PlatformInfo
    (cl:cons ':header (header msg))
    (cl:cons ':model (model msg))
    (cl:cons ':revision (revision msg))
    (cl:cons ':serial (serial msg))
))
