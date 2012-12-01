; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude VisualLocation-request.msg.html

(cl:defclass <VisualLocation-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (attempts
    :reader attempts
    :initarg :attempts
    :type cl:fixnum
    :initform 0))
)

(cl:defclass VisualLocation-request (<VisualLocation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualLocation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualLocation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<VisualLocation-request> is deprecated: use re_srvs-srv:VisualLocation-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <VisualLocation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:name-val is deprecated.  Use re_srvs-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'attempts-val :lambda-list '(m))
(cl:defmethod attempts-val ((m <VisualLocation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:attempts-val is deprecated.  Use re_srvs-srv:attempts instead.")
  (attempts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualLocation-request>) ostream)
  "Serializes a message object of type '<VisualLocation-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let* ((signed (cl:slot-value msg 'attempts)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualLocation-request>) istream)
  "Deserializes a message object of type '<VisualLocation-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'attempts) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualLocation-request>)))
  "Returns string type for a service object of type '<VisualLocation-request>"
  "re_srvs/VisualLocationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualLocation-request)))
  "Returns string type for a service object of type 'VisualLocation-request"
  "re_srvs/VisualLocationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualLocation-request>)))
  "Returns md5sum for a message object of type '<VisualLocation-request>"
  "2fe0fe9f589bf604b428cdddd534bba1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualLocation-request)))
  "Returns md5sum for a message object of type 'VisualLocation-request"
  "2fe0fe9f589bf604b428cdddd534bba1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualLocation-request>)))
  "Returns full string definition for message of type '<VisualLocation-request>"
  (cl:format cl:nil "string name~%int8 attempts~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualLocation-request)))
  "Returns full string definition for message of type 'VisualLocation-request"
  (cl:format cl:nil "string name~%int8 attempts~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualLocation-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualLocation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualLocation-request
    (cl:cons ':name (name msg))
    (cl:cons ':attempts (attempts msg))
))
;//! \htmlinclude VisualLocation-response.msg.html

(cl:defclass <VisualLocation-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass VisualLocation-response (<VisualLocation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualLocation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualLocation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<VisualLocation-response> is deprecated: use re_srvs-srv:VisualLocation-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <VisualLocation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:result-val is deprecated.  Use re_srvs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualLocation-response>) ostream)
  "Serializes a message object of type '<VisualLocation-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualLocation-response>) istream)
  "Deserializes a message object of type '<VisualLocation-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualLocation-response>)))
  "Returns string type for a service object of type '<VisualLocation-response>"
  "re_srvs/VisualLocationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualLocation-response)))
  "Returns string type for a service object of type 'VisualLocation-response"
  "re_srvs/VisualLocationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualLocation-response>)))
  "Returns md5sum for a message object of type '<VisualLocation-response>"
  "2fe0fe9f589bf604b428cdddd534bba1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualLocation-response)))
  "Returns md5sum for a message object of type 'VisualLocation-response"
  "2fe0fe9f589bf604b428cdddd534bba1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualLocation-response>)))
  "Returns full string definition for message of type '<VisualLocation-response>"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualLocation-response)))
  "Returns full string definition for message of type 'VisualLocation-response"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualLocation-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualLocation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualLocation-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'VisualLocation)))
  'VisualLocation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'VisualLocation)))
  'VisualLocation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualLocation)))
  "Returns string type for a service object of type '<VisualLocation>"
  "re_srvs/VisualLocation")