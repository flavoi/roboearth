; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude GetEnvironmentBinaryFile-request.msg.html

(cl:defclass <GetEnvironmentBinaryFile-request> (roslisp-msg-protocol:ros-message)
  ((envUID
    :reader envUID
    :initarg :envUID
    :type cl:string
    :initform "")
   (filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass GetEnvironmentBinaryFile-request (<GetEnvironmentBinaryFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetEnvironmentBinaryFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetEnvironmentBinaryFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<GetEnvironmentBinaryFile-request> is deprecated: use re_srvs-srv:GetEnvironmentBinaryFile-request instead.")))

(cl:ensure-generic-function 'envUID-val :lambda-list '(m))
(cl:defmethod envUID-val ((m <GetEnvironmentBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:envUID-val is deprecated.  Use re_srvs-srv:envUID instead.")
  (envUID m))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <GetEnvironmentBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:filename-val is deprecated.  Use re_srvs-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetEnvironmentBinaryFile-request>) ostream)
  "Serializes a message object of type '<GetEnvironmentBinaryFile-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetEnvironmentBinaryFile-request>) istream)
  "Deserializes a message object of type '<GetEnvironmentBinaryFile-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetEnvironmentBinaryFile-request>)))
  "Returns string type for a service object of type '<GetEnvironmentBinaryFile-request>"
  "re_srvs/GetEnvironmentBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEnvironmentBinaryFile-request)))
  "Returns string type for a service object of type 'GetEnvironmentBinaryFile-request"
  "re_srvs/GetEnvironmentBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetEnvironmentBinaryFile-request>)))
  "Returns md5sum for a message object of type '<GetEnvironmentBinaryFile-request>"
  "5ee8ef47af201972a853e50a0a644f44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetEnvironmentBinaryFile-request)))
  "Returns md5sum for a message object of type 'GetEnvironmentBinaryFile-request"
  "5ee8ef47af201972a853e50a0a644f44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetEnvironmentBinaryFile-request>)))
  "Returns full string definition for message of type '<GetEnvironmentBinaryFile-request>"
  (cl:format cl:nil "string envUID~%string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetEnvironmentBinaryFile-request)))
  "Returns full string definition for message of type 'GetEnvironmentBinaryFile-request"
  (cl:format cl:nil "string envUID~%string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetEnvironmentBinaryFile-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'envUID))
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetEnvironmentBinaryFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetEnvironmentBinaryFile-request
    (cl:cons ':envUID (envUID msg))
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude GetEnvironmentBinaryFile-response.msg.html

(cl:defclass <GetEnvironmentBinaryFile-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (file
    :reader file
    :initarg :file
    :type re_msgs-msg:File
    :initform (cl:make-instance 're_msgs-msg:File)))
)

(cl:defclass GetEnvironmentBinaryFile-response (<GetEnvironmentBinaryFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetEnvironmentBinaryFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetEnvironmentBinaryFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<GetEnvironmentBinaryFile-response> is deprecated: use re_srvs-srv:GetEnvironmentBinaryFile-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetEnvironmentBinaryFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <GetEnvironmentBinaryFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:file-val is deprecated.  Use re_srvs-srv:file instead.")
  (file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetEnvironmentBinaryFile-response>) ostream)
  "Serializes a message object of type '<GetEnvironmentBinaryFile-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'file) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetEnvironmentBinaryFile-response>) istream)
  "Deserializes a message object of type '<GetEnvironmentBinaryFile-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'file) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetEnvironmentBinaryFile-response>)))
  "Returns string type for a service object of type '<GetEnvironmentBinaryFile-response>"
  "re_srvs/GetEnvironmentBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEnvironmentBinaryFile-response)))
  "Returns string type for a service object of type 'GetEnvironmentBinaryFile-response"
  "re_srvs/GetEnvironmentBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetEnvironmentBinaryFile-response>)))
  "Returns md5sum for a message object of type '<GetEnvironmentBinaryFile-response>"
  "5ee8ef47af201972a853e50a0a644f44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetEnvironmentBinaryFile-response)))
  "Returns md5sum for a message object of type 'GetEnvironmentBinaryFile-response"
  "5ee8ef47af201972a853e50a0a644f44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetEnvironmentBinaryFile-response>)))
  "Returns full string definition for message of type '<GetEnvironmentBinaryFile-response>"
  (cl:format cl:nil "bool success~%re_msgs/File file~%~%~%================================================================================~%MSG: re_msgs/File~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetEnvironmentBinaryFile-response)))
  "Returns full string definition for message of type 'GetEnvironmentBinaryFile-response"
  (cl:format cl:nil "bool success~%re_msgs/File file~%~%~%================================================================================~%MSG: re_msgs/File~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetEnvironmentBinaryFile-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetEnvironmentBinaryFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetEnvironmentBinaryFile-response
    (cl:cons ':success (success msg))
    (cl:cons ':file (file msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetEnvironmentBinaryFile)))
  'GetEnvironmentBinaryFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetEnvironmentBinaryFile)))
  'GetEnvironmentBinaryFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEnvironmentBinaryFile)))
  "Returns string type for a service object of type '<GetEnvironmentBinaryFile>"
  "re_srvs/GetEnvironmentBinaryFile")