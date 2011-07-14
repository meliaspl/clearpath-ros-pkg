; Auto-generated. Do not edit!


(cl:in-package chameleon_msgs-msg)


;//! \htmlinclude ChameleonCmdSpeed.msg.html

(cl:defclass <ChameleonCmdSpeed> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:float
    :initform 0.0)
   (right
    :reader right
    :initarg :right
    :type cl:float
    :initform 0.0)
   (accel
    :reader accel
    :initarg :accel
    :type cl:float
    :initform 0.0))
)

(cl:defclass ChameleonCmdSpeed (<ChameleonCmdSpeed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChameleonCmdSpeed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChameleonCmdSpeed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chameleon_msgs-msg:<ChameleonCmdSpeed> is deprecated: use chameleon_msgs-msg:ChameleonCmdSpeed instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <ChameleonCmdSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:left-val is deprecated.  Use chameleon_msgs-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <ChameleonCmdSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:right-val is deprecated.  Use chameleon_msgs-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'accel-val :lambda-list '(m))
(cl:defmethod accel-val ((m <ChameleonCmdSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:accel-val is deprecated.  Use chameleon_msgs-msg:accel instead.")
  (accel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChameleonCmdSpeed>) ostream)
  "Serializes a message object of type '<ChameleonCmdSpeed>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'accel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChameleonCmdSpeed>) istream)
  "Deserializes a message object of type '<ChameleonCmdSpeed>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'accel) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChameleonCmdSpeed>)))
  "Returns string type for a message object of type '<ChameleonCmdSpeed>"
  "chameleon_msgs/ChameleonCmdSpeed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChameleonCmdSpeed)))
  "Returns string type for a message object of type 'ChameleonCmdSpeed"
  "chameleon_msgs/ChameleonCmdSpeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChameleonCmdSpeed>)))
  "Returns md5sum for a message object of type '<ChameleonCmdSpeed>"
  "ce72fd92980148517a294d53b7465e77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChameleonCmdSpeed)))
  "Returns md5sum for a message object of type 'ChameleonCmdSpeed"
  "ce72fd92980148517a294d53b7465e77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChameleonCmdSpeed>)))
  "Returns full string definition for message of type '<ChameleonCmdSpeed>"
  (cl:format cl:nil "float32 left   # m/s~%float32 right  # m/s~%float32 accel  # m/s^2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChameleonCmdSpeed)))
  "Returns full string definition for message of type 'ChameleonCmdSpeed"
  (cl:format cl:nil "float32 left   # m/s~%float32 right  # m/s~%float32 accel  # m/s^2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChameleonCmdSpeed>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChameleonCmdSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'ChameleonCmdSpeed
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
    (cl:cons ':accel (accel msg))
))
