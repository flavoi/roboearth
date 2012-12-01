; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude QueryActionRecipes-request.msg.html

(cl:defclass <QueryActionRecipes-request> (roslisp-msg-protocol:ros-message)
  ((query
    :reader query
    :initarg :query
    :type cl:string
    :initform ""))
)

(cl:defclass QueryActionRecipes-request (<QueryActionRecipes-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryActionRecipes-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryActionRecipes-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<QueryActionRecipes-request> is deprecated: use re_srvs-srv:QueryActionRecipes-request instead.")))

(cl:ensure-generic-function 'query-val :lambda-list '(m))
(cl:defmethod query-val ((m <QueryActionRecipes-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:query-val is deprecated.  Use re_srvs-srv:query instead.")
  (query m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryActionRecipes-request>) ostream)
  "Serializes a message object of type '<QueryActionRecipes-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'query))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'query))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryActionRecipes-request>) istream)
  "Deserializes a message object of type '<QueryActionRecipes-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryActionRecipes-request>)))
  "Returns string type for a service object of type '<QueryActionRecipes-request>"
  "re_srvs/QueryActionRecipesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryActionRecipes-request)))
  "Returns string type for a service object of type 'QueryActionRecipes-request"
  "re_srvs/QueryActionRecipesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryActionRecipes-request>)))
  "Returns md5sum for a message object of type '<QueryActionRecipes-request>"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryActionRecipes-request)))
  "Returns md5sum for a message object of type 'QueryActionRecipes-request"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryActionRecipes-request>)))
  "Returns full string definition for message of type '<QueryActionRecipes-request>"
  (cl:format cl:nil "string query~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryActionRecipes-request)))
  "Returns full string definition for message of type 'QueryActionRecipes-request"
  (cl:format cl:nil "string query~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryActionRecipes-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'query))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryActionRecipes-request>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryActionRecipes-request
    (cl:cons ':query (query msg))
))
;//! \htmlinclude QueryActionRecipes-response.msg.html

(cl:defclass <QueryActionRecipes-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform ""))
)

(cl:defclass QueryActionRecipes-response (<QueryActionRecipes-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryActionRecipes-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryActionRecipes-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<QueryActionRecipes-response> is deprecated: use re_srvs-srv:QueryActionRecipes-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <QueryActionRecipes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:result-val is deprecated.  Use re_srvs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryActionRecipes-response>) ostream)
  "Serializes a message object of type '<QueryActionRecipes-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryActionRecipes-response>) istream)
  "Deserializes a message object of type '<QueryActionRecipes-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryActionRecipes-response>)))
  "Returns string type for a service object of type '<QueryActionRecipes-response>"
  "re_srvs/QueryActionRecipesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryActionRecipes-response)))
  "Returns string type for a service object of type 'QueryActionRecipes-response"
  "re_srvs/QueryActionRecipesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryActionRecipes-response>)))
  "Returns md5sum for a message object of type '<QueryActionRecipes-response>"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryActionRecipes-response)))
  "Returns md5sum for a message object of type 'QueryActionRecipes-response"
  "40ece397ad679f27203bff340007bd45")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryActionRecipes-response>)))
  "Returns full string definition for message of type '<QueryActionRecipes-response>"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryActionRecipes-response)))
  "Returns full string definition for message of type 'QueryActionRecipes-response"
  (cl:format cl:nil "string result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryActionRecipes-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryActionRecipes-response>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryActionRecipes-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'QueryActionRecipes)))
  'QueryActionRecipes-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'QueryActionRecipes)))
  'QueryActionRecipes-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryActionRecipes)))
  "Returns string type for a service object of type '<QueryActionRecipes>"
  "re_srvs/QueryActionRecipes")