; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude QueryEnvironments-request.msg.html

(cl:defclass <QueryEnvironments-request> (roslisp-msg-protocol:ros-message)
  ((query
    :reader query
    :initarg :query
    :type cl:string
    :initform ""))
)

(cl:defclass QueryEnvironments-request (<QueryEnvironments-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryEnvironments-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryEnvironments-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<QueryEnvironments-request> is deprecated: use re_srvs-srv:QueryEnvironments-request instead.")))

(cl:ensure-generic-function 'query-val :lambda-list '(m))
(cl:defmethod query-val ((m <QueryEnvironments-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:query-val is deprecated.  Use re_srvs-srv:query instead.")
  (query m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryEnvironments-request>) ostream)
  "Serializes a message object of type '<QueryEnvironments-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'query))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'query))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryEnvironments-request>) istream)
  "Deserializes a message object of type '<QueryEnvironments-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryEnvironments-request>)))
  "Returns string type for a service object of type '<QueryEnvironments-request>"
  "re_srvs/QueryEnvironmentsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryEnvironments-request)))
  "Returns string type for a service object of type 'QueryEnvironments-request"
  "re_srvs/QueryEnvironmentsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryEnvironments-request>)))
  "Returns md5sum for a message object of type '<QueryEnvironments-request>"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryEnvironments-request)))
  "Returns md5sum for a message object of type 'QueryEnvironments-request"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryEnvironments-request>)))
  "Returns full string definition for message of type '<QueryEnvironments-request>"
  (cl:format cl:nil "string query~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryEnvironments-request)))
  "Returns full string definition for message of type 'QueryEnvironments-request"
  (cl:format cl:nil "string query~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryEnvironments-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'query))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryEnvironments-request>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryEnvironments-request
    (cl:cons ':query (query msg))
))
;//! \htmlinclude QueryEnvironments-response.msg.html

(cl:defclass <QueryEnvironments-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass QueryEnvironments-response (<QueryEnvironments-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryEnvironments-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryEnvironments-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<QueryEnvironments-response> is deprecated: use re_srvs-srv:QueryEnvironments-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <QueryEnvironments-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:result-val is deprecated.  Use re_srvs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryEnvironments-response>) ostream)
  "Serializes a message object of type '<QueryEnvironments-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryEnvironments-response>) istream)
  "Deserializes a message object of type '<QueryEnvironments-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryEnvironments-response>)))
  "Returns string type for a service object of type '<QueryEnvironments-response>"
  "re_srvs/QueryEnvironmentsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryEnvironments-response)))
  "Returns string type for a service object of type 'QueryEnvironments-response"
  "re_srvs/QueryEnvironmentsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryEnvironments-response>)))
  "Returns md5sum for a message object of type '<QueryEnvironments-response>"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryEnvironments-response)))
  "Returns md5sum for a message object of type 'QueryEnvironments-response"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryEnvironments-response>)))
  "Returns full string definition for message of type '<QueryEnvironments-response>"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryEnvironments-response)))
  "Returns full string definition for message of type 'QueryEnvironments-response"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryEnvironments-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryEnvironments-response>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryEnvironments-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'QueryEnvironments)))
  'QueryEnvironments-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'QueryEnvironments)))
  'QueryEnvironments-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryEnvironments)))
  "Returns string type for a service object of type '<QueryEnvironments>"
  "re_srvs/QueryEnvironments")