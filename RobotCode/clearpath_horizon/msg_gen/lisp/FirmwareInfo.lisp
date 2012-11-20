; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude FirmwareInfo.msg.html

(cl:defclass <FirmwareInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (firmware_major
    :reader firmware_major
    :initarg :firmware_major
    :type cl:fixnum
    :initform 0)
   (firmware_minor
    :reader firmware_minor
    :initarg :firmware_minor
    :type cl:fixnum
    :initform 0)
   (protocol_major
    :reader protocol_major
    :initarg :protocol_major
    :type cl:fixnum
    :initform 0)
   (protocol_minor
    :reader protocol_minor
    :initarg :protocol_minor
    :type cl:fixnum
    :initform 0)
   (firmware_write_time
    :reader firmware_write_time
    :initarg :firmware_write_time
    :type cl:integer
    :initform 0))
)

(cl:defclass FirmwareInfo (<FirmwareInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FirmwareInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FirmwareInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<FirmwareInfo> is deprecated: use clearpath_horizon-msg:FirmwareInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FirmwareInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:header-val is deprecated.  Use clearpath_horizon-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'firmware_major-val :lambda-list '(m))
(cl:defmethod firmware_major-val ((m <FirmwareInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:firmware_major-val is deprecated.  Use clearpath_horizon-msg:firmware_major instead.")
  (firmware_major m))

(cl:ensure-generic-function 'firmware_minor-val :lambda-list '(m))
(cl:defmethod firmware_minor-val ((m <FirmwareInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:firmware_minor-val is deprecated.  Use clearpath_horizon-msg:firmware_minor instead.")
  (firmware_minor m))

(cl:ensure-generic-function 'protocol_major-val :lambda-list '(m))
(cl:defmethod protocol_major-val ((m <FirmwareInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:protocol_major-val is deprecated.  Use clearpath_horizon-msg:protocol_major instead.")
  (protocol_major m))

(cl:ensure-generic-function 'protocol_minor-val :lambda-list '(m))
(cl:defmethod protocol_minor-val ((m <FirmwareInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:protocol_minor-val is deprecated.  Use clearpath_horizon-msg:protocol_minor instead.")
  (protocol_minor m))

(cl:ensure-generic-function 'firmware_write_time-val :lambda-list '(m))
(cl:defmethod firmware_write_time-val ((m <FirmwareInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:firmware_write_time-val is deprecated.  Use clearpath_horizon-msg:firmware_write_time instead.")
  (firmware_write_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FirmwareInfo>) ostream)
  "Serializes a message object of type '<FirmwareInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'firmware_major)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'firmware_minor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'protocol_major)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'protocol_minor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'firmware_write_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'firmware_write_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'firmware_write_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'firmware_write_time)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FirmwareInfo>) istream)
  "Deserializes a message object of type '<FirmwareInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'firmware_major) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'firmware_minor) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'protocol_major) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'protocol_minor) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'firmware_write_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'firmware_write_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'firmware_write_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'firmware_write_time)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FirmwareInfo>)))
  "Returns string type for a message object of type '<FirmwareInfo>"
  "clearpath_horizon/FirmwareInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FirmwareInfo)))
  "Returns string type for a message object of type 'FirmwareInfo"
  "clearpath_horizon/FirmwareInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FirmwareInfo>)))
  "Returns md5sum for a message object of type '<FirmwareInfo>"
  "dd399eb9c7b3816e8bea664a45a7e9ea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FirmwareInfo)))
  "Returns md5sum for a message object of type 'FirmwareInfo"
  "dd399eb9c7b3816e8bea664a45a7e9ea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FirmwareInfo>)))
  "Returns full string definition for message of type '<FirmwareInfo>"
  (cl:format cl:nil "Header header~%int8 firmware_major~%int8 firmware_minor~%int8 protocol_major~%int8 protocol_minor~%uint32 firmware_write_time~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FirmwareInfo)))
  "Returns full string definition for message of type 'FirmwareInfo"
  (cl:format cl:nil "Header header~%int8 firmware_major~%int8 firmware_minor~%int8 protocol_major~%int8 protocol_minor~%uint32 firmware_write_time~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FirmwareInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FirmwareInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'FirmwareInfo
    (cl:cons ':header (header msg))
    (cl:cons ':firmware_major (firmware_major msg))
    (cl:cons ':firmware_minor (firmware_minor msg))
    (cl:cons ':protocol_major (protocol_major msg))
    (cl:cons ':protocol_minor (protocol_minor msg))
    (cl:cons ':firmware_write_time (firmware_write_time msg))
))
