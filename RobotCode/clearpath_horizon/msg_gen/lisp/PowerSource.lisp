; Auto-generated. Do not edit!


(cl:in-package clearpath_horizon-msg)


;//! \htmlinclude PowerSource.msg.html

(cl:defclass <PowerSource> (roslisp-msg-protocol:ros-message)
  ((charge
    :reader charge
    :initarg :charge
    :type cl:float
    :initform 0.0)
   (capacity
    :reader capacity
    :initarg :capacity
    :type cl:fixnum
    :initform 0)
   (present
    :reader present
    :initarg :present
    :type cl:boolean
    :initform cl:nil)
   (in_use
    :reader in_use
    :initarg :in_use
    :type cl:boolean
    :initform cl:nil)
   (description
    :reader description
    :initarg :description
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PowerSource (<PowerSource>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PowerSource>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PowerSource)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name clearpath_horizon-msg:<PowerSource> is deprecated: use clearpath_horizon-msg:PowerSource instead.")))

(cl:ensure-generic-function 'charge-val :lambda-list '(m))
(cl:defmethod charge-val ((m <PowerSource>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:charge-val is deprecated.  Use clearpath_horizon-msg:charge instead.")
  (charge m))

(cl:ensure-generic-function 'capacity-val :lambda-list '(m))
(cl:defmethod capacity-val ((m <PowerSource>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:capacity-val is deprecated.  Use clearpath_horizon-msg:capacity instead.")
  (capacity m))

(cl:ensure-generic-function 'present-val :lambda-list '(m))
(cl:defmethod present-val ((m <PowerSource>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:present-val is deprecated.  Use clearpath_horizon-msg:present instead.")
  (present m))

(cl:ensure-generic-function 'in_use-val :lambda-list '(m))
(cl:defmethod in_use-val ((m <PowerSource>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:in_use-val is deprecated.  Use clearpath_horizon-msg:in_use instead.")
  (in_use m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <PowerSource>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader clearpath_horizon-msg:description-val is deprecated.  Use clearpath_horizon-msg:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PowerSource>) ostream)
  "Serializes a message object of type '<PowerSource>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'charge))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'capacity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'present) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'in_use) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'description)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PowerSource>) istream)
  "Deserializes a message object of type '<PowerSource>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'charge) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'capacity) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'present) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'in_use) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'description)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PowerSource>)))
  "Returns string type for a message object of type '<PowerSource>"
  "clearpath_horizon/PowerSource")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PowerSource)))
  "Returns string type for a message object of type 'PowerSource"
  "clearpath_horizon/PowerSource")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PowerSource>)))
  "Returns md5sum for a message object of type '<PowerSource>"
  "adbe384d7d69a337a7f2b6bf1d0139cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PowerSource)))
  "Returns md5sum for a message object of type 'PowerSource"
  "adbe384d7d69a337a7f2b6bf1d0139cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PowerSource>)))
  "Returns full string definition for message of type '<PowerSource>"
  (cl:format cl:nil "float32 charge~%int16 capacity~%bool present~%bool in_use~%uint8 description~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PowerSource)))
  "Returns full string definition for message of type 'PowerSource"
  (cl:format cl:nil "float32 charge~%int16 capacity~%bool present~%bool in_use~%uint8 description~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PowerSource>))
  (cl:+ 0
     4
     2
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PowerSource>))
  "Converts a ROS message object to a list"
  (cl:list 'PowerSource
    (cl:cons ':charge (charge msg))
    (cl:cons ':capacity (capacity msg))
    (cl:cons ':present (present msg))
    (cl:cons ':in_use (in_use msg))
    (cl:cons ':description (description msg))
))
