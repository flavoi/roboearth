; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude Request2DMap-request.msg.html

(cl:defclass <Request2DMap-request> (roslisp-msg-protocol:ros-message)
  ((envUID
    :reader envUID
    :initarg :envUID
    :type cl:string
    :initform "")
   (srdl
    :reader srdl
    :initarg :srdl
    :type cl:string
    :initform "")
   (baseScannerLink
    :reader baseScannerLink
    :initarg :baseScannerLink
    :type cl:string
    :initform "")
   (targetMapName
    :reader targetMapName
    :initarg :targetMapName
    :type cl:string
    :initform ""))
)

(cl:defclass Request2DMap-request (<Request2DMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Request2DMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Request2DMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<Request2DMap-request> is deprecated: use re_srvs-srv:Request2DMap-request instead.")))

(cl:ensure-generic-function 'envUID-val :lambda-list '(m))
(cl:defmethod envUID-val ((m <Request2DMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:envUID-val is deprecated.  Use re_srvs-srv:envUID instead.")
  (envUID m))

(cl:ensure-generic-function 'srdl-val :lambda-list '(m))
(cl:defmethod srdl-val ((m <Request2DMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:srdl-val is deprecated.  Use re_srvs-srv:srdl instead.")
  (srdl m))

(cl:ensure-generic-function 'baseScannerLink-val :lambda-list '(m))
(cl:defmethod baseScannerLink-val ((m <Request2DMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:baseScannerLink-val is deprecated.  Use re_srvs-srv:baseScannerLink instead.")
  (baseScannerLink m))

(cl:ensure-generic-function 'targetMapName-val :lambda-list '(m))
(cl:defmethod targetMapName-val ((m <Request2DMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:targetMapName-val is deprecated.  Use re_srvs-srv:targetMapName instead.")
  (targetMapName m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Request2DMap-request>) ostream)
  "Serializes a message object of type '<Request2DMap-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'envUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'envUID))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'srdl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'srdl))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'baseScannerLink))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'baseScannerLink))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'targetMapName))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'targetMapName))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Request2DMap-request>) istream)
  "Deserializes a message object of type '<Request2DMap-request>"
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
      (cl:setf (cl:slot-value msg 'srdl) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'srdl) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'baseScannerLink) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'baseScannerLink) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Request2DMap-request>)))
  "Returns string type for a service object of type '<Request2DMap-request>"
  "re_srvs/Request2DMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Request2DMap-request)))
  "Returns string type for a service object of type 'Request2DMap-request"
  "re_srvs/Request2DMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Request2DMap-request>)))
  "Returns md5sum for a message object of type '<Request2DMap-request>"
  "36589988f0eaacd3eb9e9e443f14ac19")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Request2DMap-request)))
  "Returns md5sum for a message object of type 'Request2DMap-request"
  "36589988f0eaacd3eb9e9e443f14ac19")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Request2DMap-request>)))
  "Returns full string definition for message of type '<Request2DMap-request>"
  (cl:format cl:nil "string envUID~%string srdl~%string baseScannerLink~%~%string targetMapName~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Request2DMap-request)))
  "Returns full string definition for message of type 'Request2DMap-request"
  (cl:format cl:nil "string envUID~%string srdl~%string baseScannerLink~%~%string targetMapName~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Request2DMap-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'envUID))
     4 (cl:length (cl:slot-value msg 'srdl))
     4 (cl:length (cl:slot-value msg 'baseScannerLink))
     4 (cl:length (cl:slot-value msg 'targetMapName))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Request2DMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Request2DMap-request
    (cl:cons ':envUID (envUID msg))
    (cl:cons ':srdl (srdl msg))
    (cl:cons ':baseScannerLink (baseScannerLink msg))
    (cl:cons ':targetMapName (targetMapName msg))
))
;//! \htmlinclude Request2DMap-response.msg.html

(cl:defclass <Request2DMap-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Request2DMap-response (<Request2DMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Request2DMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Request2DMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<Request2DMap-response> is deprecated: use re_srvs-srv:Request2DMap-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Request2DMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <Request2DMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:map-val is deprecated.  Use re_srvs-srv:map instead.")
  (map m))

(cl:ensure-generic-function 'meta-val :lambda-list '(m))
(cl:defmethod meta-val ((m <Request2DMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:meta-val is deprecated.  Use re_srvs-srv:meta instead.")
  (meta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Request2DMap-response>) ostream)
  "Serializes a message object of type '<Request2DMap-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'map) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'meta) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Request2DMap-response>) istream)
  "Deserializes a message object of type '<Request2DMap-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'map) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'meta) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Request2DMap-response>)))
  "Returns string type for a service object of type '<Request2DMap-response>"
  "re_srvs/Request2DMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Request2DMap-response)))
  "Returns string type for a service object of type 'Request2DMap-response"
  "re_srvs/Request2DMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Request2DMap-response>)))
  "Returns md5sum for a message object of type '<Request2DMap-response>"
  "36589988f0eaacd3eb9e9e443f14ac19")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Request2DMap-response)))
  "Returns md5sum for a message object of type 'Request2DMap-response"
  "36589988f0eaacd3eb9e9e443f14ac19")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Request2DMap-response>)))
  "Returns full string definition for message of type '<Request2DMap-response>"
  (cl:format cl:nil "bool success~%re_msgs/RosFile map~%re_msgs/RosFile meta~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Request2DMap-response)))
  "Returns full string definition for message of type 'Request2DMap-response"
  (cl:format cl:nil "bool success~%re_msgs/RosFile map~%re_msgs/RosFile meta~%~%~%================================================================================~%MSG: re_msgs/RosFile~%# This file representation is used to pass binary data to the RoboEarthDB.~%# As the endianess isn't stored, only files with a byte order mark (BOM) or~%# an implicitly specified endianess should be transferred.~%string name   # file name~%int8[] data   # binary data ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Request2DMap-response>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'map))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'meta))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Request2DMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Request2DMap-response
    (cl:cons ':success (success msg))
    (cl:cons ':map (map msg))
    (cl:cons ':meta (meta msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Request2DMap)))
  'Request2DMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Request2DMap)))
  'Request2DMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Request2DMap)))
  "Returns string type for a service object of type '<Request2DMap>"
  "re_srvs/Request2DMap")