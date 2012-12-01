; Auto-generated. Do not edit!


(cl:in-package re_2dmap_extractor-srv)


;//! \htmlinclude RequestNavMap-request.msg.html

(cl:defclass <RequestNavMap-request> (roslisp-msg-protocol:ros-message)
  ((octoMap
    :reader octoMap
    :initarg :octoMap
    :type re_msgs-msg:RosFile
    :initform (cl:make-instance 're_msgs-msg:RosFile))
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

(cl:defclass RequestNavMap-request (<RequestNavMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestNavMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestNavMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_2dmap_extractor-srv:<RequestNavMap-request> is deprecated: use re_2dmap_extractor-srv:RequestNavMap-request instead.")))

(cl:ensure-generic-function 'octoMap-val :lambda-list '(m))
(cl:defmethod octoMap-val ((m <RequestNavMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:octoMap-val is deprecated.  Use re_2dmap_extractor-srv:octoMap instead.")
  (octoMap m))

(cl:ensure-generic-function 'minZ-val :lambda-list '(m))
(cl:defmethod minZ-val ((m <RequestNavMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:minZ-val is deprecated.  Use re_2dmap_extractor-srv:minZ instead.")
  (minZ m))

(cl:ensure-generic-function 'maxZ-val :lambda-list '(m))
(cl:defmethod maxZ-val ((m <RequestNavMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:maxZ-val is deprecated.  Use re_2dmap_extractor-srv:maxZ instead.")
  (maxZ m))

(cl:ensure-generic-function 'targetMapName-val :lambda-list '(m))
(cl:defmethod targetMapName-val ((m <RequestNavMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:targetMapName-val is deprecated.  Use re_2dmap_extractor-srv:targetMapName instead.")
  (targetMapName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestNavMap-request>) ostream)
  "Serializes a message object of type '<RequestNavMap-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'octoMap) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestNavMap-request>) istream)
  "Deserializes a message object of type '<RequestNavMap-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'octoMap) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestNavMap-request>)))
  "Returns string type for a service object of type '<RequestNavMap-request>"
  "re_2dmap_extractor/RequestNavMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestNavMap-request)))
  "Returns string type for a service object of type 'RequestNavMap-request"
  "re_2dmap_extractor/RequestNavMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestNavMap-request>)))
  "Returns md5sum for a message object of type '<RequestNavMap-request>"
  "fc2ec8e525a85e5d84266ccaa7e6d291")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestNavMap-request)))
  "Returns md5sum for a message object of type 'RequestNavMap-request"
  "fc2ec8e525a85e5d84266ccaa7e6d291")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestNavMap-request>)))
  "Returns full string definition for message of type '<RequestNavMap-request>"
  (cl:format cl:nil "re_msgs/RosFile octoMap~%float64 minZ~%float64 maxZ~%string targetMapName~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestNavMap-request)))
  "Returns full string definition for message of type 'RequestNavMap-request"
  (cl:format cl:nil "re_msgs/RosFile octoMap~%float64 minZ~%float64 maxZ~%string targetMapName~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestNavMap-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'octoMap))
     8
     8
     4 (cl:length (cl:slot-value msg 'targetMapName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestNavMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestNavMap-request
    (cl:cons ':octoMap (octoMap msg))
    (cl:cons ':minZ (minZ msg))
    (cl:cons ':maxZ (maxZ msg))
    (cl:cons ':targetMapName (targetMapName msg))
))
;//! \htmlinclude RequestNavMap-response.msg.html

(cl:defclass <RequestNavMap-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (navMap
    :reader navMap
    :initarg :navMap
    :type re_msgs-msg:RosFile
    :initform (cl:make-instance 're_msgs-msg:RosFile))
   (navMeta
    :reader navMeta
    :initarg :navMeta
    :type re_msgs-msg:RosFile
    :initform (cl:make-instance 're_msgs-msg:RosFile)))
)

(cl:defclass RequestNavMap-response (<RequestNavMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestNavMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestNavMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_2dmap_extractor-srv:<RequestNavMap-response> is deprecated: use re_2dmap_extractor-srv:RequestNavMap-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RequestNavMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:success-val is deprecated.  Use re_2dmap_extractor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'navMap-val :lambda-list '(m))
(cl:defmethod navMap-val ((m <RequestNavMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:navMap-val is deprecated.  Use re_2dmap_extractor-srv:navMap instead.")
  (navMap m))

(cl:ensure-generic-function 'navMeta-val :lambda-list '(m))
(cl:defmethod navMeta-val ((m <RequestNavMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:navMeta-val is deprecated.  Use re_2dmap_extractor-srv:navMeta instead.")
  (navMeta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestNavMap-response>) ostream)
  "Serializes a message object of type '<RequestNavMap-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'navMap) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'navMeta) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestNavMap-response>) istream)
  "Deserializes a message object of type '<RequestNavMap-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'navMap) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'navMeta) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestNavMap-response>)))
  "Returns string type for a service object of type '<RequestNavMap-response>"
  "re_2dmap_extractor/RequestNavMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestNavMap-response)))
  "Returns string type for a service object of type 'RequestNavMap-response"
  "re_2dmap_extractor/RequestNavMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestNavMap-response>)))
  "Returns md5sum for a message object of type '<RequestNavMap-response>"
  "fc2ec8e525a85e5d84266ccaa7e6d291")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestNavMap-response)))
  "Returns md5sum for a message object of type 'RequestNavMap-response"
  "fc2ec8e525a85e5d84266ccaa7e6d291")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestNavMap-response>)))
  "Returns full string definition for message of type '<RequestNavMap-response>"
  (cl:format cl:nil "bool success~%re_msgs/RosFile navMap~%re_msgs/RosFile navMeta~%~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestNavMap-response)))
  "Returns full string definition for message of type 'RequestNavMap-response"
  (cl:format cl:nil "bool success~%re_msgs/RosFile navMap~%re_msgs/RosFile navMeta~%~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestNavMap-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'navMap))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'navMeta))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestNavMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestNavMap-response
    (cl:cons ':success (success msg))
    (cl:cons ':navMap (navMap msg))
    (cl:cons ':navMeta (navMeta msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RequestNavMap)))
  'RequestNavMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RequestNavMap)))
  'RequestNavMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestNavMap)))
  "Returns string type for a service object of type '<RequestNavMap>"
  "re_2dmap_extractor/RequestNavMap")