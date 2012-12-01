; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude DelEnvironmentBinaryFile-request.msg.html

(cl:defclass <DelEnvironmentBinaryFile-request> (roslisp-msg-protocol:ros-message)
  ((envUID
    :reader envUID
    :initarg :envUID
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

(cl:defclass DelEnvironmentBinaryFile-request (<DelEnvironmentBinaryFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DelEnvironmentBinaryFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DelEnvironmentBinaryFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DelEnvironmentBinaryFile-request> is deprecated: use re_srvs-srv:DelEnvironmentBinaryFile-request instead.")))

(cl:ensure-generic-function 'envUID-val :lambda-list '(m))
(cl:defmethod envUID-val ((m <DelEnvironmentBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:envUID-val is deprecated.  Use re_srvs-srv:envUID instead.")
  (envUID m))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <DelEnvironmentBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:filename-val is deprecated.  Use re_srvs-srv:filename instead.")
  (filename m))

(cl:ensure-generic-function 'apiKey-val :lambda-list '(m))
(cl:defmethod apiKey-val ((m <DelEnvironmentBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:apiKey-val is deprecated.  Use re_srvs-srv:apiKey instead.")
  (apiKey m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DelEnvironmentBinaryFile-request>) ostream)
  "Serializes a message object of type '<DelEnvironmentBinaryFile-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'envUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'envUID))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DelEnvironmentBinaryFile-request>) istream)
  "Deserializes a message object of type '<DelEnvironmentBinaryFile-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'envUID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'envUID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DelEnvironmentBinaryFile-request>)))
  "Returns string type for a service object of type '<DelEnvironmentBinaryFile-request>"
  "re_srvs/DelEnvironmentBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelEnvironmentBinaryFile-request)))
  "Returns string type for a service object of type 'DelEnvironmentBinaryFile-request"
  "re_srvs/DelEnvironmentBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DelEnvironmentBinaryFile-request>)))
  "Returns md5sum for a message object of type '<DelEnvironmentBinaryFile-request>"
  "92f0f09bf0f8863b94cf28ac651d722e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DelEnvironmentBinaryFile-request)))
  "Returns md5sum for a message object of type 'DelEnvironmentBinaryFile-request"
  "92f0f09bf0f8863b94cf28ac651d722e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DelEnvironmentBinaryFile-request>)))
  "Returns full string definition for message of type '<DelEnvironmentBinaryFile-request>"
  (cl:format cl:nil "string envUID~%string filename~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DelEnvironmentBinaryFile-request)))
  "Returns full string definition for message of type 'DelEnvironmentBinaryFile-request"
  (cl:format cl:nil "string envUID~%string filename~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DelEnvironmentBinaryFile-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'envUID))
     4 (cl:length (cl:slot-value msg 'filename))
     4 (cl:length (cl:slot-value msg 'apiKey))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DelEnvironmentBinaryFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DelEnvironmentBinaryFile-request
    (cl:cons ':envUID (envUID msg))
    (cl:cons ':filename (filename msg))
    (cl:cons ':apiKey (apiKey msg))
))
;//! \htmlinclude DelEnvironmentBinaryFile-response.msg.html

(cl:defclass <DelEnvironmentBinaryFile-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DelEnvironmentBinaryFile-response (<DelEnvironmentBinaryFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DelEnvironmentBinaryFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DelEnvironmentBinaryFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DelEnvironmentBinaryFile-response> is deprecated: use re_srvs-srv:DelEnvironmentBinaryFile-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DelEnvironmentBinaryFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DelEnvironmentBinaryFile-response>) ostream)
  "Serializes a message object of type '<DelEnvironmentBinaryFile-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DelEnvironmentBinaryFile-response>) istream)
  "Deserializes a message object of type '<DelEnvironmentBinaryFile-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DelEnvironmentBinaryFile-response>)))
  "Returns string type for a service object of type '<DelEnvironmentBinaryFile-response>"
  "re_srvs/DelEnvironmentBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelEnvironmentBinaryFile-response)))
  "Returns string type for a service object of type 'DelEnvironmentBinaryFile-response"
  "re_srvs/DelEnvironmentBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DelEnvironmentBinaryFile-response>)))
  "Returns md5sum for a message object of type '<DelEnvironmentBinaryFile-response>"
  "92f0f09bf0f8863b94cf28ac651d722e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DelEnvironmentBinaryFile-response)))
  "Returns md5sum for a message object of type 'DelEnvironmentBinaryFile-response"
  "92f0f09bf0f8863b94cf28ac651d722e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DelEnvironmentBinaryFile-response>)))
  "Returns full string definition for message of type '<DelEnvironmentBinaryFile-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DelEnvironmentBinaryFile-response)))
  "Returns full string definition for message of type 'DelEnvironmentBinaryFile-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DelEnvironmentBinaryFile-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DelEnvironmentBinaryFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DelEnvironmentBinaryFile-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DelEnvironmentBinaryFile)))
  'DelEnvironmentBinaryFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DelEnvironmentBinaryFile)))
  'DelEnvironmentBinaryFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelEnvironmentBinaryFile)))
  "Returns string type for a service object of type '<DelEnvironmentBinaryFile>"
  "re_srvs/DelEnvironmentBinaryFile")