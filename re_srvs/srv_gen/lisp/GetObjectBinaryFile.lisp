; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude GetObjectBinaryFile-request.msg.html

(cl:defclass <GetObjectBinaryFile-request> (roslisp-msg-protocol:ros-message)
  ((objectUID
    :reader objectUID
    :initarg :objectUID
    :type cl:string
    :initform "")
   (filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass GetObjectBinaryFile-request (<GetObjectBinaryFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetObjectBinaryFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetObjectBinaryFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<GetObjectBinaryFile-request> is deprecated: use re_srvs-srv:GetObjectBinaryFile-request instead.")))

(cl:ensure-generic-function 'objectUID-val :lambda-list '(m))
(cl:defmethod objectUID-val ((m <GetObjectBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:objectUID-val is deprecated.  Use re_srvs-srv:objectUID instead.")
  (objectUID m))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <GetObjectBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:filename-val is deprecated.  Use re_srvs-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetObjectBinaryFile-request>) ostream)
  "Serializes a message object of type '<GetObjectBinaryFile-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetObjectBinaryFile-request>) istream)
  "Deserializes a message object of type '<GetObjectBinaryFile-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetObjectBinaryFile-request>)))
  "Returns string type for a service object of type '<GetObjectBinaryFile-request>"
  "re_srvs/GetObjectBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObjectBinaryFile-request)))
  "Returns string type for a service object of type 'GetObjectBinaryFile-request"
  "re_srvs/GetObjectBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetObjectBinaryFile-request>)))
  "Returns md5sum for a message object of type '<GetObjectBinaryFile-request>"
  "2473ef6c049951863c0edac0b25e3ced")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetObjectBinaryFile-request)))
  "Returns md5sum for a message object of type 'GetObjectBinaryFile-request"
  "2473ef6c049951863c0edac0b25e3ced")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetObjectBinaryFile-request>)))
  "Returns full string definition for message of type '<GetObjectBinaryFile-request>"
  (cl:format cl:nil "string objectUID~%string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetObjectBinaryFile-request)))
  "Returns full string definition for message of type 'GetObjectBinaryFile-request"
  (cl:format cl:nil "string objectUID~%string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetObjectBinaryFile-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'objectUID))
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetObjectBinaryFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetObjectBinaryFile-request
    (cl:cons ':objectUID (objectUID msg))
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude GetObjectBinaryFile-response.msg.html

(cl:defclass <GetObjectBinaryFile-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GetObjectBinaryFile-response (<GetObjectBinaryFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetObjectBinaryFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetObjectBinaryFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<GetObjectBinaryFile-response> is deprecated: use re_srvs-srv:GetObjectBinaryFile-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetObjectBinaryFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <GetObjectBinaryFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:file-val is deprecated.  Use re_srvs-srv:file instead.")
  (file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetObjectBinaryFile-response>) ostream)
  "Serializes a message object of type '<GetObjectBinaryFile-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'file) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetObjectBinaryFile-response>) istream)
  "Deserializes a message object of type '<GetObjectBinaryFile-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'file) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetObjectBinaryFile-response>)))
  "Returns string type for a service object of type '<GetObjectBinaryFile-response>"
  "re_srvs/GetObjectBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObjectBinaryFile-response)))
  "Returns string type for a service object of type 'GetObjectBinaryFile-response"
  "re_srvs/GetObjectBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetObjectBinaryFile-response>)))
  "Returns md5sum for a message object of type '<GetObjectBinaryFile-response>"
  "2473ef6c049951863c0edac0b25e3ced")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetObjectBinaryFile-response)))
  "Returns md5sum for a message object of type 'GetObjectBinaryFile-response"
  "2473ef6c049951863c0edac0b25e3ced")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetObjectBinaryFile-response>)))
  "Returns full string definition for message of type '<GetObjectBinaryFile-response>"
  (cl:format cl:nil "bool success~%re_msgs/File file~%~%~%================================================================================~%MSG: re_msgs/File~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetObjectBinaryFile-response)))
  "Returns full string definition for message of type 'GetObjectBinaryFile-response"
  (cl:format cl:nil "bool success~%re_msgs/File file~%~%~%================================================================================~%MSG: re_msgs/File~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetObjectBinaryFile-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetObjectBinaryFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetObjectBinaryFile-response
    (cl:cons ':success (success msg))
    (cl:cons ':file (file msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetObjectBinaryFile)))
  'GetObjectBinaryFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetObjectBinaryFile)))
  'GetObjectBinaryFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObjectBinaryFile)))
  "Returns string type for a service object of type '<GetObjectBinaryFile>"
  "re_srvs/GetObjectBinaryFile")