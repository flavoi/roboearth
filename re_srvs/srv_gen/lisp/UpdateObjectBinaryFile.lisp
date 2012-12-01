; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude UpdateObjectBinaryFile-request.msg.html

(cl:defclass <UpdateObjectBinaryFile-request> (roslisp-msg-protocol:ros-message)
  ((objectUID
    :reader objectUID
    :initarg :objectUID
    :type cl:string
    :initform "")
   (file
    :reader file
    :initarg :file
    :type re_msgs-msg:File
    :initform (cl:make-instance 're_msgs-msg:File))
   (apiKey
    :reader apiKey
    :initarg :apiKey
    :type cl:string
    :initform ""))
)

(cl:defclass UpdateObjectBinaryFile-request (<UpdateObjectBinaryFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateObjectBinaryFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateObjectBinaryFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<UpdateObjectBinaryFile-request> is deprecated: use re_srvs-srv:UpdateObjectBinaryFile-request instead.")))

(cl:ensure-generic-function 'objectUID-val :lambda-list '(m))
(cl:defmethod objectUID-val ((m <UpdateObjectBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:objectUID-val is deprecated.  Use re_srvs-srv:objectUID instead.")
  (objectUID m))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <UpdateObjectBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:file-val is deprecated.  Use re_srvs-srv:file instead.")
  (file m))

(cl:ensure-generic-function 'apiKey-val :lambda-list '(m))
(cl:defmethod apiKey-val ((m <UpdateObjectBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:apiKey-val is deprecated.  Use re_srvs-srv:apiKey instead.")
  (apiKey m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateObjectBinaryFile-request>) ostream)
  "Serializes a message object of type '<UpdateObjectBinaryFile-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'objectUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'objectUID))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'file) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'apiKey))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'apiKey))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateObjectBinaryFile-request>) istream)
  "Deserializes a message object of type '<UpdateObjectBinaryFile-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'objectUID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'objectUID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'file) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateObjectBinaryFile-request>)))
  "Returns string type for a service object of type '<UpdateObjectBinaryFile-request>"
  "re_srvs/UpdateObjectBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateObjectBinaryFile-request)))
  "Returns string type for a service object of type 'UpdateObjectBinaryFile-request"
  "re_srvs/UpdateObjectBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateObjectBinaryFile-request>)))
  "Returns md5sum for a message object of type '<UpdateObjectBinaryFile-request>"
  "7808c6e49a19c0dc4fd0244bedf75931")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateObjectBinaryFile-request)))
  "Returns md5sum for a message object of type 'UpdateObjectBinaryFile-request"
  "7808c6e49a19c0dc4fd0244bedf75931")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateObjectBinaryFile-request>)))
  "Returns full string definition for message of type '<UpdateObjectBinaryFile-request>"
  (cl:format cl:nil "string objectUID~%re_msgs/File file~%string apiKey~%~%~%================================================================================~%MSG: re_msgs/File~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateObjectBinaryFile-request)))
  "Returns full string definition for message of type 'UpdateObjectBinaryFile-request"
  (cl:format cl:nil "string objectUID~%re_msgs/File file~%string apiKey~%~%~%================================================================================~%MSG: re_msgs/File~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateObjectBinaryFile-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'objectUID))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'file))
     4 (cl:length (cl:slot-value msg 'apiKey))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateObjectBinaryFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateObjectBinaryFile-request
    (cl:cons ':objectUID (objectUID msg))
    (cl:cons ':file (file msg))
    (cl:cons ':apiKey (apiKey msg))
))
;//! \htmlinclude UpdateObjectBinaryFile-response.msg.html

(cl:defclass <UpdateObjectBinaryFile-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UpdateObjectBinaryFile-response (<UpdateObjectBinaryFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateObjectBinaryFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateObjectBinaryFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<UpdateObjectBinaryFile-response> is deprecated: use re_srvs-srv:UpdateObjectBinaryFile-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <UpdateObjectBinaryFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateObjectBinaryFile-response>) ostream)
  "Serializes a message object of type '<UpdateObjectBinaryFile-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateObjectBinaryFile-response>) istream)
  "Deserializes a message object of type '<UpdateObjectBinaryFile-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateObjectBinaryFile-response>)))
  "Returns string type for a service object of type '<UpdateObjectBinaryFile-response>"
  "re_srvs/UpdateObjectBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateObjectBinaryFile-response)))
  "Returns string type for a service object of type 'UpdateObjectBinaryFile-response"
  "re_srvs/UpdateObjectBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateObjectBinaryFile-response>)))
  "Returns md5sum for a message object of type '<UpdateObjectBinaryFile-response>"
  "7808c6e49a19c0dc4fd0244bedf75931")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateObjectBinaryFile-response)))
  "Returns md5sum for a message object of type 'UpdateObjectBinaryFile-response"
  "7808c6e49a19c0dc4fd0244bedf75931")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateObjectBinaryFile-response>)))
  "Returns full string definition for message of type '<UpdateObjectBinaryFile-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateObjectBinaryFile-response)))
  "Returns full string definition for message of type 'UpdateObjectBinaryFile-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateObjectBinaryFile-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateObjectBinaryFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateObjectBinaryFile-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateObjectBinaryFile)))
  'UpdateObjectBinaryFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateObjectBinaryFile)))
  'UpdateObjectBinaryFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateObjectBinaryFile)))
  "Returns string type for a service object of type '<UpdateObjectBinaryFile>"
  "re_srvs/UpdateObjectBinaryFile")