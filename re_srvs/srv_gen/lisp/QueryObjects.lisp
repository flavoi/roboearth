; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude QueryObjects-request.msg.html

(cl:defclass <QueryObjects-request> (roslisp-msg-protocol:ros-message)
  ((query
    :reader query
    :initarg :query
    :type cl:string
    :initform ""))
)

(cl:defclass QueryObjects-request (<QueryObjects-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryObjects-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryObjects-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<QueryObjects-request> is deprecated: use re_srvs-srv:QueryObjects-request instead.")))

(cl:ensure-generic-function 'query-val :lambda-list '(m))
(cl:defmethod query-val ((m <QueryObjects-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:query-val is deprecated.  Use re_srvs-srv:query instead.")
  (query m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryObjects-request>) ostream)
  "Serializes a message object of type '<QueryObjects-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'query))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'query))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryObjects-request>) istream)
  "Deserializes a message object of type '<QueryObjects-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'query) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'query) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryObjects-request>)))
  "Returns string type for a service object of type '<QueryObjects-request>"
  "re_srvs/QueryObjectsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryObjects-request)))
  "Returns string type for a service object of type 'QueryObjects-request"
  "re_srvs/QueryObjectsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryObjects-request>)))
  "Returns md5sum for a message object of type '<QueryObjects-request>"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryObjects-request)))
  "Returns md5sum for a message object of type 'QueryObjects-request"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryObjects-request>)))
  "Returns full string definition for message of type '<QueryObjects-request>"
  (cl:format cl:nil "string query~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryObjects-request)))
  "Returns full string definition for message of type 'QueryObjects-request"
  (cl:format cl:nil "string query~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryObjects-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'query))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryObjects-request>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryObjects-request
    (cl:cons ':query (query msg))
))
;//! \htmlinclude QueryObjects-response.msg.html

(cl:defclass <QueryObjects-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass QueryObjects-response (<QueryObjects-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryObjects-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryObjects-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<QueryObjects-response> is deprecated: use re_srvs-srv:QueryObjects-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <QueryObjects-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:result-val is deprecated.  Use re_srvs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryObjects-response>) ostream)
  "Serializes a message object of type '<QueryObjects-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryObjects-response>) istream)
  "Deserializes a message object of type '<QueryObjects-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryObjects-response>)))
  "Returns string type for a service object of type '<QueryObjects-response>"
  "re_srvs/QueryObjectsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryObjects-response)))
  "Returns string type for a service object of type 'QueryObjects-response"
  "re_srvs/QueryObjectsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryObjects-response>)))
  "Returns md5sum for a message object of type '<QueryObjects-response>"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryObjects-response)))
  "Returns md5sum for a message object of type 'QueryObjects-response"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryObjects-response>)))
  "Returns full string definition for message of type '<QueryObjects-response>"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryObjects-response)))
  "Returns full string definition for message of type 'QueryObjects-response"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryObjects-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryObjects-response>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryObjects-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'QueryObjects)))
  'QueryObjects-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'QueryObjects)))
  'QueryObjects-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryObjects)))
  "Returns string type for a service object of type '<QueryObjects>"
  "re_srvs/QueryObjects")