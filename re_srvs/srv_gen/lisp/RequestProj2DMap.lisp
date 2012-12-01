; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude RequestProj2DMap-request.msg.html

(cl:defclass <RequestProj2DMap-request> (roslisp-msg-protocol:ros-message)
  ((envUID
    :reader envUID
    :initarg :envUID
    :type cl:string
    :initform "")
   (minZ
    :reader minZ
    :initarg :minZ
    :type cl:float
    :initform 0.0)
   (maxZ
    :reader maxZ
    :initarg :maxZ
    :type cl:float
    :initform 0.0)
   (targetMapName
    :reader targetMapName
    :initarg :targetMapName
    :type cl:string
    :initform ""))
)

(cl:defclass RequestProj2DMap-request (<RequestProj2DMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestProj2DMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestProj2DMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<RequestProj2DMap-request> is deprecated: use re_srvs-srv:RequestProj2DMap-request instead.")))

(cl:ensure-generic-function 'envUID-val :lambda-list '(m))
(cl:defmethod envUID-val ((m <RequestProj2DMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:envUID-val is deprecated.  Use re_srvs-srv:envUID instead.")
  (envUID m))

(cl:ensure-generic-function 'minZ-val :lambda-list '(m))
(cl:defmethod minZ-val ((m <RequestProj2DMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:minZ-val is deprecated.  Use re_srvs-srv:minZ instead.")
  (minZ m))

(cl:ensure-generic-function 'maxZ-val :lambda-list '(m))
(cl:defmethod maxZ-val ((m <RequestProj2DMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:maxZ-val is deprecated.  Use re_srvs-srv:maxZ instead.")
  (maxZ m))

(cl:ensure-generic-function 'targetMapName-val :lambda-list '(m))
(cl:defmethod targetMapName-val ((m <RequestProj2DMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:targetMapName-val is deprecated.  Use re_srvs-srv:targetMapName instead.")
  (targetMapName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestProj2DMap-request>) ostream)
  "Serializes a message object of type '<RequestProj2DMap-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'envUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'envUID))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'minZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'maxZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'targetMapName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'targetMapName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestProj2DMap-request>) istream)
  "Deserializes a message object of type '<RequestProj2DMap-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'envUID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'envUID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'minZ) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxZ) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'targetMapName) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'targetMapName) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestProj2DMap-request>)))
  "Returns string type for a service object of type '<RequestProj2DMap-request>"
  "re_srvs/RequestProj2DMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestProj2DMap-request)))
  "Returns string type for a service object of type 'RequestProj2DMap-request"
  "re_srvs/RequestProj2DMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestProj2DMap-request>)))
  "Returns md5sum for a message object of type '<RequestProj2DMap-request>"
  "e0f2eef827fa0f11b6c20a347206ed51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestProj2DMap-request)))
  "Returns md5sum for a message object of type 'RequestProj2DMap-request"
  "e0f2eef827fa0f11b6c20a347206ed51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestProj2DMap-request>)))
  "Returns full string definition for message of type '<RequestProj2DMap-request>"
  (cl:format cl:nil "string envUID~%float64 minZ~%float64 maxZ~%string targetMapName~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestProj2DMap-request)))
  "Returns full string definition for message of type 'RequestProj2DMap-request"
  (cl:format cl:nil "string envUID~%float64 minZ~%float64 maxZ~%string targetMapName~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestProj2DMap-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'envUID))
     8
     8
     4 (cl:length (cl:slot-value msg 'targetMapName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestProj2DMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestProj2DMap-request
    (cl:cons ':envUID (envUID msg))
    (cl:cons ':minZ (minZ msg))
    (cl:cons ':maxZ (maxZ msg))
    (cl:cons ':targetMapName (targetMapName msg))
))
;//! \htmlinclude RequestProj2DMap-response.msg.html

(cl:defclass <RequestProj2DMap-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (map
    :reader map
    :initarg :map
    :type re_msgs-msg:RosFile
    :initform (cl:make-instance 're_msgs-msg:RosFile))
   (meta
    :reader meta
    :initarg :meta
    :type re_msgs-msg:RosFile
    :initform (cl:make-instance 're_msgs-msg:RosFile)))
)

(cl:defclass RequestProj2DMap-response (<RequestProj2DMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestProj2DMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestProj2DMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<RequestProj2DMap-response> is deprecated: use re_srvs-srv:RequestProj2DMap-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RequestProj2DMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <RequestProj2DMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:map-val is deprecated.  Use re_srvs-srv:map instead.")
  (map m))

(cl:ensure-generic-function 'meta-val :lambda-list '(m))
(cl:defmethod meta-val ((m <RequestProj2DMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:meta-val is deprecated.  Use re_srvs-srv:meta instead.")
  (meta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestProj2DMap-response>) ostream)
  "Serializes a message object of type '<RequestProj2DMap-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'map) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'meta) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestProj2DMap-response>) istream)
  "Deserializes a message object of type '<RequestProj2DMap-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'map) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'meta) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestProj2DMap-response>)))
  "Returns string type for a service object of type '<RequestProj2DMap-response>"
  "re_srvs/RequestProj2DMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestProj2DMap-response)))
  "Returns string type for a service object of type 'RequestProj2DMap-response"
  "re_srvs/RequestProj2DMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestProj2DMap-response>)))
  "Returns md5sum for a message object of type '<RequestProj2DMap-response>"
  "e0f2eef827fa0f11b6c20a347206ed51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestProj2DMap-response)))
  "Returns md5sum for a message object of type 'RequestProj2DMap-response"
  "e0f2eef827fa0f11b6c20a347206ed51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestProj2DMap-response>)))
  "Returns full string definition for message of type '<RequestProj2DMap-response>"
  (cl:format cl:nil "bool success~%re_msgs/RosFile map~%re_msgs/RosFile meta~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestProj2DMap-response)))
  "Returns full string definition for message of type 'RequestProj2DMap-response"
  (cl:format cl:nil "bool success~%re_msgs/RosFile map~%re_msgs/RosFile meta~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestProj2DMap-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'map))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'meta))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestProj2DMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestProj2DMap-response
    (cl:cons ':success (success msg))
    (cl:cons ':map (map msg))
    (cl:cons ':meta (meta msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RequestProj2DMap)))
  'RequestProj2DMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RequestProj2DMap)))
  'RequestProj2DMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestProj2DMap)))
  "Returns string type for a service object of type '<RequestProj2DMap>"
  "re_srvs/RequestProj2DMap")