; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude RoboEarthExportCopModel-request.msg.html

(cl:defclass <RoboEarthExportCopModel-request> (roslisp-msg-protocol:ros-message)
  ((object_id
    :reader object_id
    :initarg :object_id
    :type cl:integer
    :initform 0))
)

(cl:defclass RoboEarthExportCopModel-request (<RoboEarthExportCopModel-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RoboEarthExportCopModel-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RoboEarthExportCopModel-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<RoboEarthExportCopModel-request> is deprecated: use re_srvs-srv:RoboEarthExportCopModel-request instead.")))

(cl:ensure-generic-function 'object_id-val :lambda-list '(m))
(cl:defmethod object_id-val ((m <RoboEarthExportCopModel-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:object_id-val is deprecated.  Use re_srvs-srv:object_id instead.")
  (object_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RoboEarthExportCopModel-request>) ostream)
  "Serializes a message object of type '<RoboEarthExportCopModel-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'object_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'object_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RoboEarthExportCopModel-request>) istream)
  "Deserializes a message object of type '<RoboEarthExportCopModel-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'object_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RoboEarthExportCopModel-request>)))
  "Returns string type for a service object of type '<RoboEarthExportCopModel-request>"
  "re_srvs/RoboEarthExportCopModelRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RoboEarthExportCopModel-request)))
  "Returns string type for a service object of type 'RoboEarthExportCopModel-request"
  "re_srvs/RoboEarthExportCopModelRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RoboEarthExportCopModel-request>)))
  "Returns md5sum for a message object of type '<RoboEarthExportCopModel-request>"
  "6d8d55e47adcbc6e8b3347eeb7ef0727")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RoboEarthExportCopModel-request)))
  "Returns md5sum for a message object of type 'RoboEarthExportCopModel-request"
  "6d8d55e47adcbc6e8b3347eeb7ef0727")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RoboEarthExportCopModel-request>)))
  "Returns full string definition for message of type '<RoboEarthExportCopModel-request>"
  (cl:format cl:nil "~%~%uint64 object_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RoboEarthExportCopModel-request)))
  "Returns full string definition for message of type 'RoboEarthExportCopModel-request"
  (cl:format cl:nil "~%~%uint64 object_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RoboEarthExportCopModel-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RoboEarthExportCopModel-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RoboEarthExportCopModel-request
    (cl:cons ':object_id (object_id msg))
))
;//! \htmlinclude RoboEarthExportCopModel-response.msg.html

(cl:defclass <RoboEarthExportCopModel-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:integer
    :initform 0))
)

(cl:defclass RoboEarthExportCopModel-response (<RoboEarthExportCopModel-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RoboEarthExportCopModel-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RoboEarthExportCopModel-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<RoboEarthExportCopModel-response> is deprecated: use re_srvs-srv:RoboEarthExportCopModel-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RoboEarthExportCopModel-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RoboEarthExportCopModel-response>) ostream)
  "Serializes a message object of type '<RoboEarthExportCopModel-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'success)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RoboEarthExportCopModel-response>) istream)
  "Deserializes a message object of type '<RoboEarthExportCopModel-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'success)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RoboEarthExportCopModel-response>)))
  "Returns string type for a service object of type '<RoboEarthExportCopModel-response>"
  "re_srvs/RoboEarthExportCopModelResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RoboEarthExportCopModel-response)))
  "Returns string type for a service object of type 'RoboEarthExportCopModel-response"
  "re_srvs/RoboEarthExportCopModelResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RoboEarthExportCopModel-response>)))
  "Returns md5sum for a message object of type '<RoboEarthExportCopModel-response>"
  "6d8d55e47adcbc6e8b3347eeb7ef0727")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RoboEarthExportCopModel-response)))
  "Returns md5sum for a message object of type 'RoboEarthExportCopModel-response"
  "6d8d55e47adcbc6e8b3347eeb7ef0727")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RoboEarthExportCopModel-response>)))
  "Returns full string definition for message of type '<RoboEarthExportCopModel-response>"
  (cl:format cl:nil "byte success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RoboEarthExportCopModel-response)))
  "Returns full string definition for message of type 'RoboEarthExportCopModel-response"
  (cl:format cl:nil "byte success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RoboEarthExportCopModel-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RoboEarthExportCopModel-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RoboEarthExportCopModel-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RoboEarthExportCopModel)))
  'RoboEarthExportCopModel-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RoboEarthExportCopModel)))
  'RoboEarthExportCopModel-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RoboEarthExportCopModel)))
  "Returns string type for a service object of type '<RoboEarthExportCopModel>"
  "re_srvs/RoboEarthExportCopModel")