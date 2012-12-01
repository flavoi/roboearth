; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude LoadVslamMap-request.msg.html

(cl:defclass <LoadVslamMap-request> (roslisp-msg-protocol:ros-message)
  ((mapUID
    :reader mapUID
    :initarg :mapUID
    :type cl:string
    :initform ""))
)

(cl:defclass LoadVslamMap-request (<LoadVslamMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadVslamMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadVslamMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<LoadVslamMap-request> is deprecated: use re_srvs-srv:LoadVslamMap-request instead.")))

(cl:ensure-generic-function 'mapUID-val :lambda-list '(m))
(cl:defmethod mapUID-val ((m <LoadVslamMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:mapUID-val is deprecated.  Use re_srvs-srv:mapUID instead.")
  (mapUID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadVslamMap-request>) ostream)
  "Serializes a message object of type '<LoadVslamMap-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mapUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mapUID))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadVslamMap-request>) istream)
  "Deserializes a message object of type '<LoadVslamMap-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mapUID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mapUID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadVslamMap-request>)))
  "Returns string type for a service object of type '<LoadVslamMap-request>"
  "re_srvs/LoadVslamMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadVslamMap-request)))
  "Returns string type for a service object of type 'LoadVslamMap-request"
  "re_srvs/LoadVslamMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadVslamMap-request>)))
  "Returns md5sum for a message object of type '<LoadVslamMap-request>"
  "3dfc35c40fc8503a4b6fd8ff5668fcf2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadVslamMap-request)))
  "Returns md5sum for a message object of type 'LoadVslamMap-request"
  "3dfc35c40fc8503a4b6fd8ff5668fcf2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadVslamMap-request>)))
  "Returns full string definition for message of type '<LoadVslamMap-request>"
  (cl:format cl:nil "string mapUID~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadVslamMap-request)))
  "Returns full string definition for message of type 'LoadVslamMap-request"
  (cl:format cl:nil "string mapUID~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadVslamMap-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mapUID))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadVslamMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadVslamMap-request
    (cl:cons ':mapUID (mapUID msg))
))
;//! \htmlinclude LoadVslamMap-response.msg.html

(cl:defclass <LoadVslamMap-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LoadVslamMap-response (<LoadVslamMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LoadVslamMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LoadVslamMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<LoadVslamMap-response> is deprecated: use re_srvs-srv:LoadVslamMap-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <LoadVslamMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LoadVslamMap-response>) ostream)
  "Serializes a message object of type '<LoadVslamMap-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LoadVslamMap-response>) istream)
  "Deserializes a message object of type '<LoadVslamMap-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LoadVslamMap-response>)))
  "Returns string type for a service object of type '<LoadVslamMap-response>"
  "re_srvs/LoadVslamMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadVslamMap-response)))
  "Returns string type for a service object of type 'LoadVslamMap-response"
  "re_srvs/LoadVslamMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LoadVslamMap-response>)))
  "Returns md5sum for a message object of type '<LoadVslamMap-response>"
  "3dfc35c40fc8503a4b6fd8ff5668fcf2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LoadVslamMap-response)))
  "Returns md5sum for a message object of type 'LoadVslamMap-response"
  "3dfc35c40fc8503a4b6fd8ff5668fcf2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LoadVslamMap-response>)))
  "Returns full string definition for message of type '<LoadVslamMap-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LoadVslamMap-response)))
  "Returns full string definition for message of type 'LoadVslamMap-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LoadVslamMap-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LoadVslamMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LoadVslamMap-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LoadVslamMap)))
  'LoadVslamMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LoadVslamMap)))
  'LoadVslamMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LoadVslamMap)))
  "Returns string type for a service object of type '<LoadVslamMap>"
  "re_srvs/LoadVslamMap")