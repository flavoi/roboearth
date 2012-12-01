; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude SetEnvironmentBinaryFile-request.msg.html

(cl:defclass <SetEnvironmentBinaryFile-request> (roslisp-msg-protocol:ros-message)
  ((envUID
    :reader envUID
    :initarg :envUID
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

(cl:defclass SetEnvironmentBinaryFile-request (<SetEnvironmentBinaryFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetEnvironmentBinaryFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetEnvironmentBinaryFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<SetEnvironmentBinaryFile-request> is deprecated: use re_srvs-srv:SetEnvironmentBinaryFile-request instead.")))

(cl:ensure-generic-function 'envUID-val :lambda-list '(m))
(cl:defmethod envUID-val ((m <SetEnvironmentBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:envUID-val is deprecated.  Use re_srvs-srv:envUID instead.")
  (envUID m))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <SetEnvironmentBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:file-val is deprecated.  Use re_srvs-srv:file instead.")
  (file m))

(cl:ensure-generic-function 'apiKey-val :lambda-list '(m))
(cl:defmethod apiKey-val ((m <SetEnvironmentBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:apiKey-val is deprecated.  Use re_srvs-srv:apiKey instead.")
  (apiKey m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetEnvironmentBinaryFile-request>) ostream)
  "Serializes a message object of type '<SetEnvironmentBinaryFile-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'envUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'envUID))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'file) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'apiKey))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'apiKey))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetEnvironmentBinaryFile-request>) istream)
  "Deserializes a message object of type '<SetEnvironmentBinaryFile-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'envUID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'envUID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetEnvironmentBinaryFile-request>)))
  "Returns string type for a service object of type '<SetEnvironmentBinaryFile-request>"
  "re_srvs/SetEnvironmentBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEnvironmentBinaryFile-request)))
  "Returns string type for a service object of type 'SetEnvironmentBinaryFile-request"
  "re_srvs/SetEnvironmentBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetEnvironmentBinaryFile-request>)))
  "Returns md5sum for a message object of type '<SetEnvironmentBinaryFile-request>"
  "1d33d3be5d89b549b15726c574bb237d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetEnvironmentBinaryFile-request)))
  "Returns md5sum for a message object of type 'SetEnvironmentBinaryFile-request"
  "1d33d3be5d89b549b15726c574bb237d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetEnvironmentBinaryFile-request>)))
  "Returns full string definition for message of type '<SetEnvironmentBinaryFile-request>"
  (cl:format cl:nil "string envUID~%re_msgs/File file~%string apiKey~%~%~%================================================================================~%MSG: re_msgs/File~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetEnvironmentBinaryFile-request)))
  "Returns full string definition for message of type 'SetEnvironmentBinaryFile-request"
  (cl:format cl:nil "string envUID~%re_msgs/File file~%string apiKey~%~%~%================================================================================~%MSG: re_msgs/File~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetEnvironmentBinaryFile-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'envUID))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'file))
     4 (cl:length (cl:slot-value msg 'apiKey))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetEnvironmentBinaryFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetEnvironmentBinaryFile-request
    (cl:cons ':envUID (envUID msg))
    (cl:cons ':file (file msg))
    (cl:cons ':apiKey (apiKey msg))
))
;//! \htmlinclude SetEnvironmentBinaryFile-response.msg.html

(cl:defclass <SetEnvironmentBinaryFile-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetEnvironmentBinaryFile-response (<SetEnvironmentBinaryFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetEnvironmentBinaryFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetEnvironmentBinaryFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<SetEnvironmentBinaryFile-response> is deprecated: use re_srvs-srv:SetEnvironmentBinaryFile-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetEnvironmentBinaryFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetEnvironmentBinaryFile-response>) ostream)
  "Serializes a message object of type '<SetEnvironmentBinaryFile-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetEnvironmentBinaryFile-response>) istream)
  "Deserializes a message object of type '<SetEnvironmentBinaryFile-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetEnvironmentBinaryFile-response>)))
  "Returns string type for a service object of type '<SetEnvironmentBinaryFile-response>"
  "re_srvs/SetEnvironmentBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEnvironmentBinaryFile-response)))
  "Returns string type for a service object of type 'SetEnvironmentBinaryFile-response"
  "re_srvs/SetEnvironmentBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetEnvironmentBinaryFile-response>)))
  "Returns md5sum for a message object of type '<SetEnvironmentBinaryFile-response>"
  "1d33d3be5d89b549b15726c574bb237d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetEnvironmentBinaryFile-response)))
  "Returns md5sum for a message object of type 'SetEnvironmentBinaryFile-response"
  "1d33d3be5d89b549b15726c574bb237d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetEnvironmentBinaryFile-response>)))
  "Returns full string definition for message of type '<SetEnvironmentBinaryFile-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetEnvironmentBinaryFile-response)))
  "Returns full string definition for message of type 'SetEnvironmentBinaryFile-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetEnvironmentBinaryFile-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetEnvironmentBinaryFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetEnvironmentBinaryFile-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetEnvironmentBinaryFile)))
  'SetEnvironmentBinaryFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetEnvironmentBinaryFile)))
  'SetEnvironmentBinaryFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEnvironmentBinaryFile)))
  "Returns string type for a service object of type '<SetEnvironmentBinaryFile>"
  "re_srvs/SetEnvironmentBinaryFile")