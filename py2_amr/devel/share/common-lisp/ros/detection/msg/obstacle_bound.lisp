; Auto-generated. Do not edit!


(cl:in-package detection-msg)


;//! \htmlinclude obstacle_bound.msg.html

(cl:defclass <obstacle_bound> (roslisp-msg-protocol:ros-message)
  ((top_left_x
    :reader top_left_x
    :initarg :top_left_x
    :type cl:fixnum
    :initform 0)
   (top_left_y
    :reader top_left_y
    :initarg :top_left_y
    :type cl:fixnum
    :initform 0)
   (bottom_right_x
    :reader bottom_right_x
    :initarg :bottom_right_x
    :type cl:fixnum
    :initform 0)
   (bottom_right_y
    :reader bottom_right_y
    :initarg :bottom_right_y
    :type cl:fixnum
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass obstacle_bound (<obstacle_bound>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obstacle_bound>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obstacle_bound)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name detection-msg:<obstacle_bound> is deprecated: use detection-msg:obstacle_bound instead.")))

(cl:ensure-generic-function 'top_left_x-val :lambda-list '(m))
(cl:defmethod top_left_x-val ((m <obstacle_bound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection-msg:top_left_x-val is deprecated.  Use detection-msg:top_left_x instead.")
  (top_left_x m))

(cl:ensure-generic-function 'top_left_y-val :lambda-list '(m))
(cl:defmethod top_left_y-val ((m <obstacle_bound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection-msg:top_left_y-val is deprecated.  Use detection-msg:top_left_y instead.")
  (top_left_y m))

(cl:ensure-generic-function 'bottom_right_x-val :lambda-list '(m))
(cl:defmethod bottom_right_x-val ((m <obstacle_bound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection-msg:bottom_right_x-val is deprecated.  Use detection-msg:bottom_right_x instead.")
  (bottom_right_x m))

(cl:ensure-generic-function 'bottom_right_y-val :lambda-list '(m))
(cl:defmethod bottom_right_y-val ((m <obstacle_bound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection-msg:bottom_right_y-val is deprecated.  Use detection-msg:bottom_right_y instead.")
  (bottom_right_y m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <obstacle_bound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detection-msg:distance-val is deprecated.  Use detection-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obstacle_bound>) ostream)
  "Serializes a message object of type '<obstacle_bound>"
  (cl:let* ((signed (cl:slot-value msg 'top_left_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'top_left_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bottom_right_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bottom_right_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obstacle_bound>) istream)
  "Deserializes a message object of type '<obstacle_bound>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'top_left_x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'top_left_y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bottom_right_x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bottom_right_y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obstacle_bound>)))
  "Returns string type for a message object of type '<obstacle_bound>"
  "detection/obstacle_bound")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacle_bound)))
  "Returns string type for a message object of type 'obstacle_bound"
  "detection/obstacle_bound")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obstacle_bound>)))
  "Returns md5sum for a message object of type '<obstacle_bound>"
  "41b15b11d68bbfaa7472618894c283f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obstacle_bound)))
  "Returns md5sum for a message object of type 'obstacle_bound"
  "41b15b11d68bbfaa7472618894c283f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obstacle_bound>)))
  "Returns full string definition for message of type '<obstacle_bound>"
  (cl:format cl:nil "int16 top_left_x~%int16 top_left_y~%int16 bottom_right_x~%int16 bottom_right_y~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obstacle_bound)))
  "Returns full string definition for message of type 'obstacle_bound"
  (cl:format cl:nil "int16 top_left_x~%int16 top_left_y~%int16 bottom_right_x~%int16 bottom_right_y~%float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obstacle_bound>))
  (cl:+ 0
     2
     2
     2
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obstacle_bound>))
  "Converts a ROS message object to a list"
  (cl:list 'obstacle_bound
    (cl:cons ':top_left_x (top_left_x msg))
    (cl:cons ':top_left_y (top_left_y msg))
    (cl:cons ':bottom_right_x (bottom_right_x msg))
    (cl:cons ':bottom_right_y (bottom_right_y msg))
    (cl:cons ':distance (distance msg))
))
