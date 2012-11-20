; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude DifferentialSpeed.msg.html

(cl:defclass <DifferentialSpeed> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left_speed
    :reader left_speed
    :initarg :left_speed
    :type cl:float
    :initform 0.0)
   (right_speed
    :reader right_speed
    :initarg :right_speed
    :type cl:float
    :initform 0.0)
   (left_accel
    :reader left_accel
    :initarg :left_accel
    :type cl:float
    :initform 0.0)
   (right_accel
    :reader right_accel
    :initarg :right_accel
    :type cl:float
    :initform 0.0))
)

(cl:defclass DifferentialSpeed (<DifferentialSpeed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DifferentialSpeed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DifferentialSpeed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<DifferentialSpeed> is deprecated: use clearpath_horizon-msg:DifferentialSpeed instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DifferentialSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:header-val is deprecated.  Use clearpath_horizon-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left_speed-val :lambda-list '(m))
(cl:defmethod left_speed-val ((m <DifferentialSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:left_speed-val is deprecated.  Use clearpath_horizon-msg:left_speed instead.")
  (left_speed m))

(cl:ensure-generic-function 'right_speed-val :lambda-list '(m))
(cl:defmethod right_speed-val ((m <DifferentialSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:right_speed-val is deprecated.  Use clearpath_horizon-msg:right_speed instead.")
  (right_speed m))

(cl:ensure-generic-function 'left_accel-val :lambda-list '(m))
(cl:defmethod left_accel-val ((m <DifferentialSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:left_accel-val is deprecated.  Use clearpath_horizon-msg:left_accel instead.")
  (left_accel m))

(cl:ensure-generic-function 'right_accel-val :lambda-list '(m))
(cl:defmethod right_accel-val ((m <DifferentialSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:right_accel-val is deprecated.  Use clearpath_horizon-msg:right_accel instead.")
  (right_accel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DifferentialSpeed>) ostream)
  "Serializes a message object of type '<DifferentialSpeed>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_accel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_accel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DifferentialSpeed>) istream)
  "Deserializes a message object of type '<DifferentialSpeed>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_accel) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_accel) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DifferentialSpeed>)))
  "Returns string type for a message object of type '<DifferentialSpeed>"
  "clearpath_horizon/DifferentialSpeed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DifferentialSpeed)))
  "Returns string type for a message object of type 'DifferentialSpeed"
  "clearpath_horizon/DifferentialSpeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DifferentialSpeed>)))
  "Returns md5sum for a message object of type '<DifferentialSpeed>"
  "4b62e763320bacb9c7f63f6ebd46ceba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DifferentialSpeed)))
  "Returns md5sum for a message object of type 'DifferentialSpeed"
  "4b62e763320bacb9c7f63f6ebd46ceba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DifferentialSpeed>)))
  "Returns full string definition for message of type '<DifferentialSpeed>"
  (cl:format cl:nil "Header header~%float64 left_speed~%float64 right_speed~%float64 left_accel~%float64 right_accel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DifferentialSpeed)))
  "Returns full string definition for message of type 'DifferentialSpeed"
  (cl:format cl:nil "Header header~%float64 left_speed~%float64 right_speed~%float64 left_accel~%float64 right_accel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DifferentialSpeed>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DifferentialSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'DifferentialSpeed
    (cl:cons ':header (header msg))
    (cl:cons ':left_speed (left_speed msg))
    (cl:cons ':right_speed (right_speed msg))
    (cl:cons ':left_accel (left_accel msg))
    (cl:cons ':right_accel (right_accel msg))
))
