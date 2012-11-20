; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude StateChange.msg.html

(cl:defclass <StateChange> (roslisp-msg-protocol:ros-message)
  ((new_state
    :reader new_state
    :initarg :new_state
    :type cl:string
    :initform "")
   (joystick
    :reader joystick
    :initarg :joystick
    :type cl:string
    :initform ""))
)

(cl:defclass StateChange (<StateChange>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateChange>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateChange)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<StateChange> is deprecated: use clearpath_horizon-msg:StateChange instead.")))

(cl:ensure-generic-function 'new_state-val :lambda-list '(m))
(cl:defmethod new_state-val ((m <StateChange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:new_state-val is deprecated.  Use clearpath_horizon-msg:new_state instead.")
  (new_state m))

(cl:ensure-generic-function 'joystick-val :lambda-list '(m))
(cl:defmethod joystick-val ((m <StateChange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:joystick-val is deprecated.  Use clearpath_horizon-msg:joystick instead.")
  (joystick m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateChange>) ostream)
  "Serializes a message object of type '<StateChange>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'new_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'new_state))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'joystick))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'joystick))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateChange>) istream)
  "Deserializes a message object of type '<StateChange>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'new_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'new_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'joystick) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'joystick) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateChange>)))
  "Returns string type for a message object of type '<StateChange>"
  "clearpath_horizon/StateChange")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateChange)))
  "Returns string type for a message object of type 'StateChange"
  "clearpath_horizon/StateChange")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateChange>)))
  "Returns md5sum for a message object of type '<StateChange>"
  "44a4273c39fe35090d35b71e32a477da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateChange)))
  "Returns md5sum for a message object of type 'StateChange"
  "44a4273c39fe35090d35b71e32a477da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateChange>)))
  "Returns full string definition for message of type '<StateChange>"
  (cl:format cl:nil "string new_state~%string joystick~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateChange)))
  "Returns full string definition for message of type 'StateChange"
  (cl:format cl:nil "string new_state~%string joystick~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateChange>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'new_state))
     4 (cl:length (cl:slot-value msg 'joystick))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateChange>))
  "Converts a ROS message object to a list"
  (cl:list 'StateChange
    (cl:cons ':new_state (new_state msg))
    (cl:cons ':joystick (joystick msg))
))
