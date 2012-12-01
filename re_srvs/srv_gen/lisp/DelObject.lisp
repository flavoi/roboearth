; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude DelObject-request.msg.html

(cl:defclass <DelObject-request> (roslisp-msg-protocol:ros-message)
  ((objectUID
    :reader objectUID
    :initarg :objectUID
    :type cl:string
    :initform "")
   (apiKey
    :reader apiKey
    :initarg :apiKey
    :type cl:string
    :initform ""))
)

(cl:defclass DelObject-request (<DelObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DelObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DelObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DelObject-request> is deprecated: use re_srvs-srv:DelObject-request instead.")))

(cl:ensure-generic-function 'objectUID-val :lambda-list '(m))
(cl:defmethod objectUID-val ((m <DelObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:objectUID-val is deprecated.  Use re_srvs-srv:objectUID instead.")
  (objectUID m))

(cl:ensure-generic-function 'apiKey-val :lambda-list '(m))
(cl:defmethod apiKey-val ((m <DelObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:apiKey-val is deprecated.  Use re_srvs-srv:apiKey instead.")
  (apiKey m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DelObject-request>) ostream)
  "Serializes a message object of type '<DelObject-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'objectUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'objectUID))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'apiKey))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'apiKey))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DelObject-request>) istream)
  "Deserializes a message object of type '<DelObject-request>"
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
      (cl:setf (cl:slot-value msg 'apiKey) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'apiKey) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DelObject-request>)))
  "Returns string type for a service object of type '<DelObject-request>"
  "re_srvs/DelObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelObject-request)))
  "Returns string type for a service object of type 'DelObject-request"
  "re_srvs/DelObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DelObject-request>)))
  "Returns md5sum for a message object of type '<DelObject-request>"
  "81ac674334c4dcfb20edcc930b734034")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DelObject-request)))
  "Returns md5sum for a message object of type 'DelObject-request"
  "81ac674334c4dcfb20edcc930b734034")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DelObject-request>)))
  "Returns full string definition for message of type '<DelObject-request>"
  (cl:format cl:nil "string objectUID~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DelObject-request)))
  "Returns full string definition for message of type 'DelObject-request"
  (cl:format cl:nil "string objectUID~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DelObject-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'objectUID))
     4 (cl:length (cl:slot-value msg 'apiKey))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DelObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DelObject-request
    (cl:cons ':objectUID (objectUID msg))
    (cl:cons ':apiKey (apiKey msg))
))
;//! \htmlinclude DelObject-response.msg.html

(cl:defclass <DelObject-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DelObject-response (<DelObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DelObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DelObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DelObject-response> is deprecated: use re_srvs-srv:DelObject-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DelObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DelObject-response>) ostream)
  "Serializes a message object of type '<DelObject-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DelObject-response>) istream)
  "Deserializes a message object of type '<DelObject-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DelObject-response>)))
  "Returns string type for a service object of type '<DelObject-response>"
  "re_srvs/DelObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelObject-response)))
  "Returns string type for a service object of type 'DelObject-response"
  "re_srvs/DelObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DelObject-response>)))
  "Returns md5sum for a message object of type '<DelObject-response>"
  "81ac674334c4dcfb20edcc930b734034")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DelObject-response)))
  "Returns md5sum for a message object of type 'DelObject-response"
  "81ac674334c4dcfb20edcc930b734034")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DelObject-response>)))
  "Returns full string definition for message of type '<DelObject-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DelObject-response)))
  "Returns full string definition for message of type 'DelObject-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DelObject-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DelObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DelObject-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DelObject)))
  'DelObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DelObject)))
  'DelObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelObject)))
  "Returns string type for a service object of type '<DelObject>"
  "re_srvs/DelObject")