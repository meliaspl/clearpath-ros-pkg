; Auto-generated. Do not edit!


(cl:in-package chameleon_msgs-msg)


;//! \htmlinclude ChameleonCmdPWM.msg.html

(cl:defclass <ChameleonCmdPWM> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:float
    :initform 0.0)
   (right
    :reader right
    :initarg :right
    :type cl:float
    :initform 0.0))
)

(cl:defclass ChameleonCmdPWM (<ChameleonCmdPWM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChameleonCmdPWM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChameleonCmdPWM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chameleon_msgs-msg:<ChameleonCmdPWM> is deprecated: use chameleon_msgs-msg:ChameleonCmdPWM instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <ChameleonCmdPWM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:left-val is deprecated.  Use chameleon_msgs-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <ChameleonCmdPWM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:right-val is deprecated.  Use chameleon_msgs-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChameleonCmdPWM>) ostream)
  "Serializes a message object of type '<ChameleonCmdPWM>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChameleonCmdPWM>) istream)
  "Deserializes a message object of type '<ChameleonCmdPWM>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChameleonCmdPWM>)))
  "Returns string type for a message object of type '<ChameleonCmdPWM>"
  "chameleon_msgs/ChameleonCmdPWM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChameleonCmdPWM)))
  "Returns string type for a message object of type 'ChameleonCmdPWM"
  "chameleon_msgs/ChameleonCmdPWM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChameleonCmdPWM>)))
  "Returns md5sum for a message object of type '<ChameleonCmdPWM>"
  "3a927990ab5d5c3d628e2d52b8533e52")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChameleonCmdPWM)))
  "Returns md5sum for a message object of type 'ChameleonCmdPWM"
  "3a927990ab5d5c3d628e2d52b8533e52")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChameleonCmdPWM>)))
  "Returns full string definition for message of type '<ChameleonCmdPWM>"
  (cl:format cl:nil "float32 left   # 0..1~%float32 right  # 0..1~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChameleonCmdPWM)))
  "Returns full string definition for message of type 'ChameleonCmdPWM"
  (cl:format cl:nil "float32 left   # 0..1~%float32 right  # 0..1~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChameleonCmdPWM>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChameleonCmdPWM>))
  "Converts a ROS message object to a list"
  (cl:list 'ChameleonCmdPWM
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
