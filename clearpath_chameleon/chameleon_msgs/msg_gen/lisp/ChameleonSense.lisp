; Auto-generated. Do not edit!


(cl:in-package chameleon_msgs-msg)


;//! \htmlinclude ChameleonSense.msg.html

(cl:defclass <ChameleonSense> (roslisp-msg-protocol:ros-message)
  ((travel_left
    :reader travel_left
    :initarg :travel_left
    :type cl:float
    :initform 0.0)
   (travel_right
    :reader travel_right
    :initarg :travel_right
    :type cl:float
    :initform 0.0)
   (speed_left
    :reader speed_left
    :initarg :speed_left
    :type cl:float
    :initform 0.0)
   (speed_right
    :reader speed_right
    :initarg :speed_right
    :type cl:float
    :initform 0.0)
   (pwm_left
    :reader pwm_left
    :initarg :pwm_left
    :type cl:float
    :initform 0.0)
   (pwm_right
    :reader pwm_right
    :initarg :pwm_right
    :type cl:float
    :initform 0.0)
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0))
)

(cl:defclass ChameleonSense (<ChameleonSense>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChameleonSense>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChameleonSense)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chameleon_msgs-msg:<ChameleonSense> is deprecated: use chameleon_msgs-msg:ChameleonSense instead.")))

(cl:ensure-generic-function 'travel_left-val :lambda-list '(m))
(cl:defmethod travel_left-val ((m <ChameleonSense>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:travel_left-val is deprecated.  Use chameleon_msgs-msg:travel_left instead.")
  (travel_left m))

(cl:ensure-generic-function 'travel_right-val :lambda-list '(m))
(cl:defmethod travel_right-val ((m <ChameleonSense>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:travel_right-val is deprecated.  Use chameleon_msgs-msg:travel_right instead.")
  (travel_right m))

(cl:ensure-generic-function 'speed_left-val :lambda-list '(m))
(cl:defmethod speed_left-val ((m <ChameleonSense>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:speed_left-val is deprecated.  Use chameleon_msgs-msg:speed_left instead.")
  (speed_left m))

(cl:ensure-generic-function 'speed_right-val :lambda-list '(m))
(cl:defmethod speed_right-val ((m <ChameleonSense>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:speed_right-val is deprecated.  Use chameleon_msgs-msg:speed_right instead.")
  (speed_right m))

(cl:ensure-generic-function 'pwm_left-val :lambda-list '(m))
(cl:defmethod pwm_left-val ((m <ChameleonSense>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:pwm_left-val is deprecated.  Use chameleon_msgs-msg:pwm_left instead.")
  (pwm_left m))

(cl:ensure-generic-function 'pwm_right-val :lambda-list '(m))
(cl:defmethod pwm_right-val ((m <ChameleonSense>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:pwm_right-val is deprecated.  Use chameleon_msgs-msg:pwm_right instead.")
  (pwm_right m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <ChameleonSense>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chameleon_msgs-msg:voltage-val is deprecated.  Use chameleon_msgs-msg:voltage instead.")
  (voltage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChameleonSense>) ostream)
  "Serializes a message object of type '<ChameleonSense>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'travel_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'travel_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pwm_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pwm_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChameleonSense>) istream)
  "Deserializes a message object of type '<ChameleonSense>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'travel_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'travel_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pwm_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pwm_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChameleonSense>)))
  "Returns string type for a message object of type '<ChameleonSense>"
  "chameleon_msgs/ChameleonSense")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChameleonSense)))
  "Returns string type for a message object of type 'ChameleonSense"
  "chameleon_msgs/ChameleonSense")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChameleonSense>)))
  "Returns md5sum for a message object of type '<ChameleonSense>"
  "c898ed18685ac0e3eece998b04aeace2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChameleonSense)))
  "Returns md5sum for a message object of type 'ChameleonSense"
  "c898ed18685ac0e3eece998b04aeace2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChameleonSense>)))
  "Returns full string definition for message of type '<ChameleonSense>"
  (cl:format cl:nil "# Encoder travel (m)~%float32 travel_left~%float32 travel_right~%~%# Encoder speed (m/s)~%float32 speed_left~%float32 speed_right~%~%# Output PWM (-1..1)~%float32 pwm_left~%float32 pwm_right~%~%# Battery (V)~%float32 voltage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChameleonSense)))
  "Returns full string definition for message of type 'ChameleonSense"
  (cl:format cl:nil "# Encoder travel (m)~%float32 travel_left~%float32 travel_right~%~%# Encoder speed (m/s)~%float32 speed_left~%float32 speed_right~%~%# Output PWM (-1..1)~%float32 pwm_left~%float32 pwm_right~%~%# Battery (V)~%float32 voltage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChameleonSense>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChameleonSense>))
  "Converts a ROS message object to a list"
  (cl:list 'ChameleonSense
    (cl:cons ':travel_left (travel_left msg))
    (cl:cons ':travel_right (travel_right msg))
    (cl:cons ':speed_left (speed_left msg))
    (cl:cons ':speed_right (speed_right msg))
    (cl:cons ':pwm_left (pwm_left msg))
    (cl:cons ':pwm_right (pwm_right msg))
    (cl:cons ':voltage (voltage msg))
))
