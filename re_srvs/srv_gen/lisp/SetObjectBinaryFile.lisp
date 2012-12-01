; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude SetObjectBinaryFile-request.msg.html

(cl:defclass <SetObjectBinaryFile-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetObjectBinaryFile-request (<SetObjectBinaryFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetObjectBinaryFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetObjectBinaryFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<SetObjectBinaryFile-request> is deprecated: use re_srvs-srv:SetObjectBinaryFile-request instead.")))

(cl:ensure-generic-function 'objectUID-val :lambda-list '(m))
(cl:defmethod objectUID-val ((m <SetObjectBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:objectUID-val is deprecated.  Use re_srvs-srv:objectUID instead.")
  (objectUID m))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <SetObjectBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:file-val is deprecated.  Use re_srvs-srv:file instead.")
  (file m))

(cl:ensure-generic-function 'apiKey-val :lambda-list '(m))
(cl:defmethod apiKey-val ((m <SetObjectBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:apiKey-val is deprecated.  Use re_srvs-srv:apiKey instead.")
  (apiKey m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetObjectBinaryFile-request>) ostream)
  "Serializes a message object of type '<SetObjectBinaryFile-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetObjectBinaryFile-request>) istream)
  "Deserializes a message object of type '<SetObjectBinaryFile-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetObjectBinaryFile-request>)))
  "Returns string type for a service object of type '<SetObjectBinaryFile-request>"
  "re_srvs/SetObjectBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetObjectBinaryFile-request)))
  "Returns string type for a service object of type 'SetObjectBinaryFile-request"
  "re_srvs/SetObjectBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetObjectBinaryFile-request>)))
  "Returns md5sum for a message object of type '<SetObjectBinaryFile-request>"
  "7808c6e49a19c0dc4fd0244bedf75931")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetObjectBinaryFile-request)))
  "Returns md5sum for a message object of type 'SetObjectBinaryFile-request"
  "7808c6e49a19c0dc4fd0244bedf75931")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetObjectBinaryFile-request>)))
  "Returns full string definition for message of type '<SetObjectBinaryFile-request>"
  (cl:format cl:nil "string objectUID~%re_msgs/File file~%string apiKey~%~%~%================================================================================~%MSG: re_msgs/File~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetObjectBinaryFile-request)))
  "Returns full string definition for message of type 'SetObjectBinaryFile-request"
  (cl:format cl:nil "string objectUID~%re_msgs/File file~%string apiKey~%~%~%================================================================================~%MSG: re_msgs/File~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetObjectBinaryFile-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'objectUID))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'file))
     4 (cl:length (cl:slot-value msg 'apiKey))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetObjectBinaryFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetObjectBinaryFile-request
    (cl:cons ':objectUID (objectUID msg))
    (cl:cons ':file (file msg))
    (cl:cons ':apiKey (apiKey msg))
))
;//! \htmlinclude SetObjectBinaryFile-response.msg.html

(cl:defclass <SetObjectBinaryFile-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetObjectBinaryFile-response (<SetObjectBinaryFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetObjectBinaryFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetObjectBinaryFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<SetObjectBinaryFile-response> is deprecated: use re_srvs-srv:SetObjectBinaryFile-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetObjectBinaryFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetObjectBinaryFile-response>) ostream)
  "Serializes a message object of type '<SetObjectBinaryFile-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetObjectBinaryFile-response>) istream)
  "Deserializes a message object of type '<SetObjectBinaryFile-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetObjectBinaryFile-response>)))
  "Returns string type for a service object of type '<SetObjectBinaryFile-response>"
  "re_srvs/SetObjectBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetObjectBinaryFile-response)))
  "Returns string type for a service object of type 'SetObjectBinaryFile-response"
  "re_srvs/SetObjectBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetObjectBinaryFile-response>)))
  "Returns md5sum for a message object of type '<SetObjectBinaryFile-response>"
  "7808c6e49a19c0dc4fd0244bedf75931")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetObjectBinaryFile-response)))
  "Returns md5sum for a message object of type 'SetObjectBinaryFile-response"
  "7808c6e49a19c0dc4fd0244bedf75931")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetObjectBinaryFile-response>)))
  "Returns full string definition for message of type '<SetObjectBinaryFile-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetObjectBinaryFile-response)))
  "Returns full string definition for message of type 'SetObjectBinaryFile-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetObjectBinaryFile-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetObjectBinaryFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetObjectBinaryFile-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetObjectBinaryFile)))
  'SetObjectBinaryFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetObjectBinaryFile)))
  'SetObjectBinaryFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetObjectBinaryFile)))
  "Returns string type for a service object of type '<SetObjectBinaryFile>"
  "re_srvs/SetObjectBinaryFile")