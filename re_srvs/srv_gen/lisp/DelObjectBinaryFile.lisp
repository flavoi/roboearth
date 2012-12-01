; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude DelObjectBinaryFile-request.msg.html

(cl:defclass <DelObjectBinaryFile-request> (roslisp-msg-protocol:ros-message)
  ((objectUID
    :reader objectUID
    :initarg :objectUID
    :type cl:string
    :initform "")
   (filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform "")
   (apiKey
    :reader apiKey
    :initarg :apiKey
    :type cl:string
    :initform ""))
)

(cl:defclass DelObjectBinaryFile-request (<DelObjectBinaryFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DelObjectBinaryFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DelObjectBinaryFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DelObjectBinaryFile-request> is deprecated: use re_srvs-srv:DelObjectBinaryFile-request instead.")))

(cl:ensure-generic-function 'objectUID-val :lambda-list '(m))
(cl:defmethod objectUID-val ((m <DelObjectBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:objectUID-val is deprecated.  Use re_srvs-srv:objectUID instead.")
  (objectUID m))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <DelObjectBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:filename-val is deprecated.  Use re_srvs-srv:filename instead.")
  (filename m))

(cl:ensure-generic-function 'apiKey-val :lambda-list '(m))
(cl:defmethod apiKey-val ((m <DelObjectBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:apiKey-val is deprecated.  Use re_srvs-srv:apiKey instead.")
  (apiKey m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DelObjectBinaryFile-request>) ostream)
  "Serializes a message object of type '<DelObjectBinaryFile-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'objectUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'objectUID))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'apiKey))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'apiKey))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DelObjectBinaryFile-request>) istream)
  "Deserializes a message object of type '<DelObjectBinaryFile-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'objectUID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'objectUID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DelObjectBinaryFile-request>)))
  "Returns string type for a service object of type '<DelObjectBinaryFile-request>"
  "re_srvs/DelObjectBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelObjectBinaryFile-request)))
  "Returns string type for a service object of type 'DelObjectBinaryFile-request"
  "re_srvs/DelObjectBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DelObjectBinaryFile-request>)))
  "Returns md5sum for a message object of type '<DelObjectBinaryFile-request>"
  "70eebf4c74c43e8287286de77b805579")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DelObjectBinaryFile-request)))
  "Returns md5sum for a message object of type 'DelObjectBinaryFile-request"
  "70eebf4c74c43e8287286de77b805579")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DelObjectBinaryFile-request>)))
  "Returns full string definition for message of type '<DelObjectBinaryFile-request>"
  (cl:format cl:nil "string objectUID~%string filename~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DelObjectBinaryFile-request)))
  "Returns full string definition for message of type 'DelObjectBinaryFile-request"
  (cl:format cl:nil "string objectUID~%string filename~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DelObjectBinaryFile-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'objectUID))
     4 (cl:length (cl:slot-value msg 'filename))
     4 (cl:length (cl:slot-value msg 'apiKey))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DelObjectBinaryFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DelObjectBinaryFile-request
    (cl:cons ':objectUID (objectUID msg))
    (cl:cons ':filename (filename msg))
    (cl:cons ':apiKey (apiKey msg))
))
;//! \htmlinclude DelObjectBinaryFile-response.msg.html

(cl:defclass <DelObjectBinaryFile-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DelObjectBinaryFile-response (<DelObjectBinaryFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DelObjectBinaryFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DelObjectBinaryFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DelObjectBinaryFile-response> is deprecated: use re_srvs-srv:DelObjectBinaryFile-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DelObjectBinaryFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DelObjectBinaryFile-response>) ostream)
  "Serializes a message object of type '<DelObjectBinaryFile-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DelObjectBinaryFile-response>) istream)
  "Deserializes a message object of type '<DelObjectBinaryFile-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DelObjectBinaryFile-response>)))
  "Returns string type for a service object of type '<DelObjectBinaryFile-response>"
  "re_srvs/DelObjectBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelObjectBinaryFile-response)))
  "Returns string type for a service object of type 'DelObjectBinaryFile-response"
  "re_srvs/DelObjectBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DelObjectBinaryFile-response>)))
  "Returns md5sum for a message object of type '<DelObjectBinaryFile-response>"
  "70eebf4c74c43e8287286de77b805579")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DelObjectBinaryFile-response)))
  "Returns md5sum for a message object of type 'DelObjectBinaryFile-response"
  "70eebf4c74c43e8287286de77b805579")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DelObjectBinaryFile-response>)))
  "Returns full string definition for message of type '<DelObjectBinaryFile-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DelObjectBinaryFile-response)))
  "Returns full string definition for message of type 'DelObjectBinaryFile-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DelObjectBinaryFile-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DelObjectBinaryFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DelObjectBinaryFile-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DelObjectBinaryFile)))
  'DelObjectBinaryFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DelObjectBinaryFile)))
  'DelObjectBinaryFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelObjectBinaryFile)))
  "Returns string type for a service object of type '<DelObjectBinaryFile>"
  "re_srvs/DelObjectBinaryFile")