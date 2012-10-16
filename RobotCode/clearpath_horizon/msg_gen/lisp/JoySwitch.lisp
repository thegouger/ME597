; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude JoySwitch.msg.html

(cl:defclass <JoySwitch> (roslisp-msg-protocol:ros-message)
  ((robot_id
    :reader robot_id
    :initarg :robot_id
    :type cl:string
    :initform "")
   (attach
    :reader attach
    :initarg :attach
    :type cl:fixnum
    :initform 0)
   (joystick
    :reader joystick
    :initarg :joystick
    :type cl:string
    :initform ""))
)

(cl:defclass JoySwitch (<JoySwitch>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JoySwitch>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JoySwitch)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<JoySwitch> is deprecated: use clearpath_horizon-msg:JoySwitch instead.")))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <JoySwitch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:robot_id-val is deprecated.  Use clearpath_horizon-msg:robot_id instead.")
  (robot_id m))

(cl:ensure-generic-function 'attach-val :lambda-list '(m))
(cl:defmethod attach-val ((m <JoySwitch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:attach-val is deprecated.  Use clearpath_horizon-msg:attach instead.")
  (attach m))

(cl:ensure-generic-function 'joystick-val :lambda-list '(m))
(cl:defmethod joystick-val ((m <JoySwitch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:joystick-val is deprecated.  Use clearpath_horizon-msg:joystick instead.")
  (joystick m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JoySwitch>) ostream)
  "Serializes a message object of type '<JoySwitch>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'attach)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'joystick))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'joystick))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JoySwitch>) istream)
  "Deserializes a message object of type '<JoySwitch>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'attach)) (cl:read-byte istream))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JoySwitch>)))
  "Returns string type for a message object of type '<JoySwitch>"
  "clearpath_horizon/JoySwitch")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JoySwitch)))
  "Returns string type for a message object of type 'JoySwitch"
  "clearpath_horizon/JoySwitch")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JoySwitch>)))
  "Returns md5sum for a message object of type '<JoySwitch>"
  "0b9e4d12a122fa671dc7b4bd8741705d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JoySwitch)))
  "Returns md5sum for a message object of type 'JoySwitch"
  "0b9e4d12a122fa671dc7b4bd8741705d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JoySwitch>)))
  "Returns full string definition for message of type '<JoySwitch>"
  (cl:format cl:nil "string robot_id~%uint8 attach~%string joystick~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JoySwitch)))
  "Returns full string definition for message of type 'JoySwitch"
  (cl:format cl:nil "string robot_id~%uint8 attach~%string joystick~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JoySwitch>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'robot_id))
     1
     4 (cl:length (cl:slot-value msg 'joystick))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JoySwitch>))
  "Converts a ROS message object to a list"
  (cl:list 'JoySwitch
    (cl:cons ':robot_id (robot_id msg))
    (cl:cons ':attach (attach msg))
    (cl:cons ':joystick (joystick msg))
))
