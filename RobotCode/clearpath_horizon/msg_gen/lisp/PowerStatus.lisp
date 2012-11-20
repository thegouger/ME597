; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude PowerStatus.msg.html

(cl:defclass <PowerStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (sources
    :reader sources
    :initarg :sources
    :type (cl:vector clearpath_horizon-msg:PowerSource)
   :initform (cl:make-array 0 :element-type 'clearpath_horizon-msg:PowerSource :initial-element (cl:make-instance 'clearpath_horizon-msg:PowerSource))))
)

(cl:defclass PowerStatus (<PowerStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PowerStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PowerStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<PowerStatus> is deprecated: use clearpath_horizon-msg:PowerStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:header-val is deprecated.  Use clearpath_horizon-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'sources-val :lambda-list '(m))
(cl:defmethod sources-val ((m <PowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:sources-val is deprecated.  Use clearpath_horizon-msg:sources instead.")
  (sources m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PowerStatus>) ostream)
  "Serializes a message object of type '<PowerStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sources))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sources))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PowerStatus>) istream)
  "Deserializes a message object of type '<PowerStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sources) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sources)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'clearpath_horizon-msg:PowerSource))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PowerStatus>)))
  "Returns string type for a message object of type '<PowerStatus>"
  "clearpath_horizon/PowerStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PowerStatus)))
  "Returns string type for a message object of type 'PowerStatus"
  "clearpath_horizon/PowerStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PowerStatus>)))
  "Returns md5sum for a message object of type '<PowerStatus>"
  "f246c359530c58415aee4fe89d1aca04")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PowerStatus)))
  "Returns md5sum for a message object of type 'PowerStatus"
  "f246c359530c58415aee4fe89d1aca04")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PowerStatus>)))
  "Returns full string definition for message of type '<PowerStatus>"
  (cl:format cl:nil "Header header~%PowerSource[] sources~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: clearpath_horizon/PowerSource~%float32 charge~%int16 capacity~%bool present~%bool in_use~%uint8 description~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PowerStatus)))
  "Returns full string definition for message of type 'PowerStatus"
  (cl:format cl:nil "Header header~%PowerSource[] sources~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: clearpath_horizon/PowerSource~%float32 charge~%int16 capacity~%bool present~%bool in_use~%uint8 description~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PowerStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sources) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PowerStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'PowerStatus
    (cl:cons ':header (header msg))
    (cl:cons ':sources (sources msg))
))
