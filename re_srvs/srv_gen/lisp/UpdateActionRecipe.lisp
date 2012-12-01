; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude UpdateActionRecipe-request.msg.html

(cl:defclass <UpdateActionRecipe-request> (roslisp-msg-protocol:ros-message)
  ((uid
    :reader uid
    :initarg :uid
    :type cl:string
    :initform "")
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform "")
   (recipe
    :reader recipe
    :initarg :recipe
    :type cl:string
    :initform "")
   (apiKey
    :reader apiKey
    :initarg :apiKey
    :type cl:string
    :initform ""))
)

(cl:defclass UpdateActionRecipe-request (<UpdateActionRecipe-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateActionRecipe-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateActionRecipe-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<UpdateActionRecipe-request> is deprecated: use re_srvs-srv:UpdateActionRecipe-request instead.")))

(cl:ensure-generic-function 'uid-val :lambda-list '(m))
(cl:defmethod uid-val ((m <UpdateActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:uid-val is deprecated.  Use re_srvs-srv:uid instead.")
  (uid m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <UpdateActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:description-val is deprecated.  Use re_srvs-srv:description instead.")
  (description m))

(cl:ensure-generic-function 'recipe-val :lambda-list '(m))
(cl:defmethod recipe-val ((m <UpdateActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:recipe-val is deprecated.  Use re_srvs-srv:recipe instead.")
  (recipe m))

(cl:ensure-generic-function 'apiKey-val :lambda-list '(m))
(cl:defmethod apiKey-val ((m <UpdateActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:apiKey-val is deprecated.  Use re_srvs-srv:apiKey instead.")
  (apiKey m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateActionRecipe-request>) ostream)
  "Serializes a message object of type '<UpdateActionRecipe-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'uid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'uid))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'recipe))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'recipe))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'apiKey))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'apiKey))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateActionRecipe-request>) istream)
  "Deserializes a message object of type '<UpdateActionRecipe-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'uid) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'uid) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'recipe) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'recipe) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateActionRecipe-request>)))
  "Returns string type for a service object of type '<UpdateActionRecipe-request>"
  "re_srvs/UpdateActionRecipeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateActionRecipe-request)))
  "Returns string type for a service object of type 'UpdateActionRecipe-request"
  "re_srvs/UpdateActionRecipeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateActionRecipe-request>)))
  "Returns md5sum for a message object of type '<UpdateActionRecipe-request>"
  "989edddef9e26f2f408dfaf8c90f75a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateActionRecipe-request)))
  "Returns md5sum for a message object of type 'UpdateActionRecipe-request"
  "989edddef9e26f2f408dfaf8c90f75a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateActionRecipe-request>)))
  "Returns full string definition for message of type '<UpdateActionRecipe-request>"
  (cl:format cl:nil "string uid~%string description~%string recipe~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateActionRecipe-request)))
  "Returns full string definition for message of type 'UpdateActionRecipe-request"
  (cl:format cl:nil "string uid~%string description~%string recipe~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateActionRecipe-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'uid))
     4 (cl:length (cl:slot-value msg 'description))
     4 (cl:length (cl:slot-value msg 'recipe))
     4 (cl:length (cl:slot-value msg 'apiKey))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateActionRecipe-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateActionRecipe-request
    (cl:cons ':uid (uid msg))
    (cl:cons ':description (description msg))
    (cl:cons ':recipe (recipe msg))
    (cl:cons ':apiKey (apiKey msg))
))
;//! \htmlinclude UpdateActionRecipe-response.msg.html

(cl:defclass <UpdateActionRecipe-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UpdateActionRecipe-response (<UpdateActionRecipe-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateActionRecipe-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateActionRecipe-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<UpdateActionRecipe-response> is deprecated: use re_srvs-srv:UpdateActionRecipe-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <UpdateActionRecipe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateActionRecipe-response>) ostream)
  "Serializes a message object of type '<UpdateActionRecipe-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateActionRecipe-response>) istream)
  "Deserializes a message object of type '<UpdateActionRecipe-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateActionRecipe-response>)))
  "Returns string type for a service object of type '<UpdateActionRecipe-response>"
  "re_srvs/UpdateActionRecipeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateActionRecipe-response)))
  "Returns string type for a service object of type 'UpdateActionRecipe-response"
  "re_srvs/UpdateActionRecipeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateActionRecipe-response>)))
  "Returns md5sum for a message object of type '<UpdateActionRecipe-response>"
  "989edddef9e26f2f408dfaf8c90f75a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateActionRecipe-response)))
  "Returns md5sum for a message object of type 'UpdateActionRecipe-response"
  "989edddef9e26f2f408dfaf8c90f75a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateActionRecipe-response>)))
  "Returns full string definition for message of type '<UpdateActionRecipe-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateActionRecipe-response)))
  "Returns full string definition for message of type 'UpdateActionRecipe-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateActionRecipe-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateActionRecipe-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateActionRecipe-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateActionRecipe)))
  'UpdateActionRecipe-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateActionRecipe)))
  'UpdateActionRecipe-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateActionRecipe)))
  "Returns string type for a service object of type '<UpdateActionRecipe>"
  "re_srvs/UpdateActionRecipe")