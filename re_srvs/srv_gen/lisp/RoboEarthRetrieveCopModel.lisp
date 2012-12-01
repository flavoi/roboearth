; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude RoboEarthRetrieveCopModel-request.msg.html

(cl:defclass <RoboEarthRetrieveCopModel-request> (roslisp-msg-protocol:ros-message)
  ((object_name
    :reader object_name
    :initarg :object_name
    :type cl:string
    :initform ""))
)

(cl:defclass RoboEarthRetrieveCopModel-request (<RoboEarthRetrieveCopModel-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RoboEarthRetrieveCopModel-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RoboEarthRetrieveCopModel-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<RoboEarthRetrieveCopModel-request> is deprecated: use re_srvs-srv:RoboEarthRetrieveCopModel-request instead.")))

(cl:ensure-generic-function 'object_name-val :lambda-list '(m))
(cl:defmethod object_name-val ((m <RoboEarthRetrieveCopModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:object_name-val is deprecated.  Use re_srvs-srv:object_name instead.")
  (object_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RoboEarthRetrieveCopModel-request>) ostream)
  "Serializes a message object of type '<RoboEarthRetrieveCopModel-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RoboEarthRetrieveCopModel-request>) istream)
  "Deserializes a message object of type '<RoboEarthRetrieveCopModel-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RoboEarthRetrieveCopModel-request>)))
  "Returns string type for a service object of type '<RoboEarthRetrieveCopModel-request>"
  "re_srvs/RoboEarthRetrieveCopModelRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RoboEarthRetrieveCopModel-request)))
  "Returns string type for a service object of type 'RoboEarthRetrieveCopModel-request"
  "re_srvs/RoboEarthRetrieveCopModelRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RoboEarthRetrieveCopModel-request>)))
  "Returns md5sum for a message object of type '<RoboEarthRetrieveCopModel-request>"
  "87d2f809d62d013732ccd3863f969fb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RoboEarthRetrieveCopModel-request)))
  "Returns md5sum for a message object of type 'RoboEarthRetrieveCopModel-request"
  "87d2f809d62d013732ccd3863f969fb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RoboEarthRetrieveCopModel-request>)))
  "Returns full string definition for message of type '<RoboEarthRetrieveCopModel-request>"
  (cl:format cl:nil "~%~%string object_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RoboEarthRetrieveCopModel-request)))
  "Returns full string definition for message of type 'RoboEarthRetrieveCopModel-request"
  (cl:format cl:nil "~%~%string object_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RoboEarthRetrieveCopModel-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'object_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RoboEarthRetrieveCopModel-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RoboEarthRetrieveCopModel-request
    (cl:cons ':object_name (object_name msg))
))
;//! \htmlinclude RoboEarthRetrieveCopModel-response.msg.html

(cl:defclass <RoboEarthRetrieveCopModel-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0)
   (owldata
    :reader owldata
    :initarg :owldata
    :type cl:string
    :initform ""))
)

(cl:defclass RoboEarthRetrieveCopModel-response (<RoboEarthRetrieveCopModel-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RoboEarthRetrieveCopModel-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RoboEarthRetrieveCopModel-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<RoboEarthRetrieveCopModel-response> is deprecated: use re_srvs-srv:RoboEarthRetrieveCopModel-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RoboEarthRetrieveCopModel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'owldata-val :lambda-list '(m))
(cl:defmethod owldata-val ((m <RoboEarthRetrieveCopModel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:owldata-val is deprecated.  Use re_srvs-srv:owldata instead.")
  (owldata m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RoboEarthRetrieveCopModel-response>) ostream)
  "Serializes a message object of type '<RoboEarthRetrieveCopModel-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'success)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'owldata))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'owldata))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RoboEarthRetrieveCopModel-response>) istream)
  "Deserializes a message object of type '<RoboEarthRetrieveCopModel-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'success)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'owldata) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'owldata) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RoboEarthRetrieveCopModel-response>)))
  "Returns string type for a service object of type '<RoboEarthRetrieveCopModel-response>"
  "re_srvs/RoboEarthRetrieveCopModelResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RoboEarthRetrieveCopModel-response)))
  "Returns string type for a service object of type 'RoboEarthRetrieveCopModel-response"
  "re_srvs/RoboEarthRetrieveCopModelResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RoboEarthRetrieveCopModel-response>)))
  "Returns md5sum for a message object of type '<RoboEarthRetrieveCopModel-response>"
  "87d2f809d62d013732ccd3863f969fb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RoboEarthRetrieveCopModel-response)))
  "Returns md5sum for a message object of type 'RoboEarthRetrieveCopModel-response"
  "87d2f809d62d013732ccd3863f969fb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RoboEarthRetrieveCopModel-response>)))
  "Returns full string definition for message of type '<RoboEarthRetrieveCopModel-response>"
  (cl:format cl:nil "byte success~%string owldata~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RoboEarthRetrieveCopModel-response)))
  "Returns full string definition for message of type 'RoboEarthRetrieveCopModel-response"
  (cl:format cl:nil "byte success~%string owldata~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RoboEarthRetrieveCopModel-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'owldata))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RoboEarthRetrieveCopModel-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RoboEarthRetrieveCopModel-response
    (cl:cons ':success (success msg))
    (cl:cons ':owldata (owldata msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RoboEarthRetrieveCopModel)))
  'RoboEarthRetrieveCopModel-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RoboEarthRetrieveCopModel)))
  'RoboEarthRetrieveCopModel-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RoboEarthRetrieveCopModel)))
  "Returns string type for a service object of type '<RoboEarthRetrieveCopModel>"
  "re_srvs/RoboEarthRetrieveCopModel")