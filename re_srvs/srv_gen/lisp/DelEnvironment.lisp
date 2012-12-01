; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude DelEnvironment-request.msg.html

(cl:defclass <DelEnvironment-request> (roslisp-msg-protocol:ros-message)
  ((environmentUID
    :reader environmentUID
    :initarg :environmentUID
    :type cl:string
    :initform "")
   (apiKey
    :reader apiKey
    :initarg :apiKey
    :type cl:string
    :initform ""))
)

(cl:defclass DelEnvironment-request (<DelEnvironment-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DelEnvironment-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DelEnvironment-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DelEnvironment-request> is deprecated: use re_srvs-srv:DelEnvironment-request instead.")))

(cl:ensure-generic-function 'environmentUID-val :lambda-list '(m))
(cl:defmethod environmentUID-val ((m <DelEnvironment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:environmentUID-val is deprecated.  Use re_srvs-srv:environmentUID instead.")
  (environmentUID m))

(cl:ensure-generic-function 'apiKey-val :lambda-list '(m))
(cl:defmethod apiKey-val ((m <DelEnvironment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:apiKey-val is deprecated.  Use re_srvs-srv:apiKey instead.")
  (apiKey m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DelEnvironment-request>) ostream)
  "Serializes a message object of type '<DelEnvironment-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'environmentUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'environmentUID))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'apiKey))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'apiKey))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DelEnvironment-request>) istream)
  "Deserializes a message object of type '<DelEnvironment-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'environmentUID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'environmentUID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'apiKey) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'apiKey) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DelEnvironment-request>)))
  "Returns string type for a service object of type '<DelEnvironment-request>"
  "re_srvs/DelEnvironmentRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelEnvironment-request)))
  "Returns string type for a service object of type 'DelEnvironment-request"
  "re_srvs/DelEnvironmentRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DelEnvironment-request>)))
  "Returns md5sum for a message object of type '<DelEnvironment-request>"
  "872ee923659735e8d49ccdf103c68241")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DelEnvironment-request)))
  "Returns md5sum for a message object of type 'DelEnvironment-request"
  "872ee923659735e8d49ccdf103c68241")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DelEnvironment-request>)))
  "Returns full string definition for message of type '<DelEnvironment-request>"
  (cl:format cl:nil "string environmentUID~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DelEnvironment-request)))
  "Returns full string definition for message of type 'DelEnvironment-request"
  (cl:format cl:nil "string environmentUID~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DelEnvironment-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'environmentUID))
     4 (cl:length (cl:slot-value msg 'apiKey))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DelEnvironment-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DelEnvironment-request
    (cl:cons ':environmentUID (environmentUID msg))
    (cl:cons ':apiKey (apiKey msg))
))
;//! \htmlinclude DelEnvironment-response.msg.html

(cl:defclass <DelEnvironment-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DelEnvironment-response (<DelEnvironment-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DelEnvironment-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DelEnvironment-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DelEnvironment-response> is deprecated: use re_srvs-srv:DelEnvironment-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DelEnvironment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DelEnvironment-response>) ostream)
  "Serializes a message object of type '<DelEnvironment-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DelEnvironment-response>) istream)
  "Deserializes a message object of type '<DelEnvironment-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DelEnvironment-response>)))
  "Returns string type for a service object of type '<DelEnvironment-response>"
  "re_srvs/DelEnvironmentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelEnvironment-response)))
  "Returns string type for a service object of type 'DelEnvironment-response"
  "re_srvs/DelEnvironmentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DelEnvironment-response>)))
  "Returns md5sum for a message object of type '<DelEnvironment-response>"
  "872ee923659735e8d49ccdf103c68241")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DelEnvironment-response)))
  "Returns md5sum for a message object of type 'DelEnvironment-response"
  "872ee923659735e8d49ccdf103c68241")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DelEnvironment-response>)))
  "Returns full string definition for message of type '<DelEnvironment-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DelEnvironment-response)))
  "Returns full string definition for message of type 'DelEnvironment-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DelEnvironment-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DelEnvironment-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DelEnvironment-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DelEnvironment)))
  'DelEnvironment-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DelEnvironment)))
  'DelEnvironment-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelEnvironment)))
  "Returns string type for a service object of type '<DelEnvironment>"
  "re_srvs/DelEnvironment")