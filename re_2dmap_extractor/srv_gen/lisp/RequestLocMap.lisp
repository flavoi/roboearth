; Auto-generated. Do not edit!


(cl:in-package re_2dmap_extractor-srv)


;//! \htmlinclude RequestLocMap-request.msg.html

(cl:defclass <RequestLocMap-request> (roslisp-msg-protocol:ros-message)
  ((octoMap
    :reader octoMap
    :initarg :octoMap
    :type re_msgs-msg:RosFile
    :initform (cl:make-instance 're_msgs-msg:RosFile))
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (targetMapName
    :reader targetMapName
    :initarg :targetMapName
    :type cl:string
    :initform ""))
)

(cl:defclass RequestLocMap-request (<RequestLocMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestLocMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestLocMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_2dmap_extractor-srv:<RequestLocMap-request> is deprecated: use re_2dmap_extractor-srv:RequestLocMap-request instead.")))

(cl:ensure-generic-function 'octoMap-val :lambda-list '(m))
(cl:defmethod octoMap-val ((m <RequestLocMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:octoMap-val is deprecated.  Use re_2dmap_extractor-srv:octoMap instead.")
  (octoMap m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <RequestLocMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:z-val is deprecated.  Use re_2dmap_extractor-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'targetMapName-val :lambda-list '(m))
(cl:defmethod targetMapName-val ((m <RequestLocMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:targetMapName-val is deprecated.  Use re_2dmap_extractor-srv:targetMapName instead.")
  (targetMapName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestLocMap-request>) ostream)
  "Serializes a message object of type '<RequestLocMap-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'octoMap) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestLocMap-request>) istream)
  "Deserializes a message object of type '<RequestLocMap-request>"
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
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestLocMap-request>)))
  "Returns string type for a service object of type '<RequestLocMap-request>"
  "re_2dmap_extractor/RequestLocMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestLocMap-request)))
  "Returns string type for a service object of type 'RequestLocMap-request"
  "re_2dmap_extractor/RequestLocMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestLocMap-request>)))
  "Returns md5sum for a message object of type '<RequestLocMap-request>"
  "ff776d365b44637e4e3b054f6c62341b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestLocMap-request)))
  "Returns md5sum for a message object of type 'RequestLocMap-request"
  "ff776d365b44637e4e3b054f6c62341b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestLocMap-request>)))
  "Returns full string definition for message of type '<RequestLocMap-request>"
  (cl:format cl:nil "re_msgs/RosFile octoMap~%float64 z~%string targetMapName~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestLocMap-request)))
  "Returns full string definition for message of type 'RequestLocMap-request"
  (cl:format cl:nil "re_msgs/RosFile octoMap~%float64 z~%string targetMapName~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestLocMap-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'octoMap))
     8
     4 (cl:length (cl:slot-value msg 'targetMapName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestLocMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestLocMap-request
    (cl:cons ':octoMap (octoMap msg))
    (cl:cons ':z (z msg))
    (cl:cons ':targetMapName (targetMapName msg))
))
;//! \htmlinclude RequestLocMap-response.msg.html

(cl:defclass <RequestLocMap-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (locMap
    :reader locMap
    :initarg :locMap
    :type re_msgs-msg:RosFile
    :initform (cl:make-instance 're_msgs-msg:RosFile))
   (locMeta
    :reader locMeta
    :initarg :locMeta
    :type re_msgs-msg:RosFile
    :initform (cl:make-instance 're_msgs-msg:RosFile)))
)

(cl:defclass RequestLocMap-response (<RequestLocMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RequestLocMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RequestLocMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_2dmap_extractor-srv:<RequestLocMap-response> is deprecated: use re_2dmap_extractor-srv:RequestLocMap-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RequestLocMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:success-val is deprecated.  Use re_2dmap_extractor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'locMap-val :lambda-list '(m))
(cl:defmethod locMap-val ((m <RequestLocMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:locMap-val is deprecated.  Use re_2dmap_extractor-srv:locMap instead.")
  (locMap m))

(cl:ensure-generic-function 'locMeta-val :lambda-list '(m))
(cl:defmethod locMeta-val ((m <RequestLocMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_2dmap_extractor-srv:locMeta-val is deprecated.  Use re_2dmap_extractor-srv:locMeta instead.")
  (locMeta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RequestLocMap-response>) ostream)
  "Serializes a message object of type '<RequestLocMap-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'locMap) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'locMeta) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RequestLocMap-response>) istream)
  "Deserializes a message object of type '<RequestLocMap-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'locMap) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'locMeta) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RequestLocMap-response>)))
  "Returns string type for a service object of type '<RequestLocMap-response>"
  "re_2dmap_extractor/RequestLocMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestLocMap-response)))
  "Returns string type for a service object of type 'RequestLocMap-response"
  "re_2dmap_extractor/RequestLocMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RequestLocMap-response>)))
  "Returns md5sum for a message object of type '<RequestLocMap-response>"
  "ff776d365b44637e4e3b054f6c62341b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RequestLocMap-response)))
  "Returns md5sum for a message object of type 'RequestLocMap-response"
  "ff776d365b44637e4e3b054f6c62341b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RequestLocMap-response>)))
  "Returns full string definition for message of type '<RequestLocMap-response>"
  (cl:format cl:nil "bool success~%re_msgs/RosFile locMap~%re_msgs/RosFile locMeta~%~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RequestLocMap-response)))
  "Returns full string definition for message of type 'RequestLocMap-response"
  (cl:format cl:nil "bool success~%re_msgs/RosFile locMap~%re_msgs/RosFile locMeta~%~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RequestLocMap-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'locMap))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'locMeta))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RequestLocMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RequestLocMap-response
    (cl:cons ':success (success msg))
    (cl:cons ':locMap (locMap msg))
    (cl:cons ':locMeta (locMeta msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RequestLocMap)))
  'RequestLocMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RequestLocMap)))
  'RequestLocMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RequestLocMap)))
  "Returns string type for a service object of type '<RequestLocMap>"
  "re_2dmap_extractor/RequestLocMap")