; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude SetActionRecipe-request.msg.html

(cl:defclass <SetActionRecipe-request> (roslisp-msg-protocol:ros-message)
  ((cls
    :reader cls
    :initarg :cls
    :type cl:string
    :initform "")
   (id
    :reader id
    :initarg :id
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

(cl:defclass SetActionRecipe-request (<SetActionRecipe-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetActionRecipe-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetActionRecipe-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<SetActionRecipe-request> is deprecated: use re_srvs-srv:SetActionRecipe-request instead.")))

(cl:ensure-generic-function 'cls-val :lambda-list '(m))
(cl:defmethod cls-val ((m <SetActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:cls-val is deprecated.  Use re_srvs-srv:cls instead.")
  (cls m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SetActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:id-val is deprecated.  Use re_srvs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <SetActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:description-val is deprecated.  Use re_srvs-srv:description instead.")
  (description m))

(cl:ensure-generic-function 'recipe-val :lambda-list '(m))
(cl:defmethod recipe-val ((m <SetActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:recipe-val is deprecated.  Use re_srvs-srv:recipe instead.")
  (recipe m))

(cl:ensure-generic-function 'apiKey-val :lambda-list '(m))
(cl:defmethod apiKey-val ((m <SetActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:apiKey-val is deprecated.  Use re_srvs-srv:apiKey instead.")
  (apiKey m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetActionRecipe-request>) ostream)
  "Serializes a message object of type '<SetActionRecipe-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cls))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cls))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetActionRecipe-request>) istream)
  "Deserializes a message object of type '<SetActionRecipe-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cls) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cls) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetActionRecipe-request>)))
  "Returns string type for a service object of type '<SetActionRecipe-request>"
  "re_srvs/SetActionRecipeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetActionRecipe-request)))
  "Returns string type for a service object of type 'SetActionRecipe-request"
  "re_srvs/SetActionRecipeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetActionRecipe-request>)))
  "Returns md5sum for a message object of type '<SetActionRecipe-request>"
  "f36f93785e5f7f843ef366910639a201")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetActionRecipe-request)))
  "Returns md5sum for a message object of type 'SetActionRecipe-request"
  "f36f93785e5f7f843ef366910639a201")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetActionRecipe-request>)))
  "Returns full string definition for message of type '<SetActionRecipe-request>"
  (cl:format cl:nil "~%string cls~%string id~%string description~%string recipe~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetActionRecipe-request)))
  "Returns full string definition for message of type 'SetActionRecipe-request"
  (cl:format cl:nil "~%string cls~%string id~%string description~%string recipe~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetActionRecipe-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cls))
     4 (cl:length (cl:slot-value msg 'id))
     4 (cl:length (cl:slot-value msg 'description))
     4 (cl:length (cl:slot-value msg 'recipe))
     4 (cl:length (cl:slot-value msg 'apiKey))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetActionRecipe-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetActionRecipe-request
    (cl:cons ':cls (cls msg))
    (cl:cons ':id (id msg))
    (cl:cons ':description (description msg))
    (cl:cons ':recipe (recipe msg))
    (cl:cons ':apiKey (apiKey msg))
))
;//! \htmlinclude SetActionRecipe-response.msg.html

(cl:defclass <SetActionRecipe-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetActionRecipe-response (<SetActionRecipe-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetActionRecipe-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetActionRecipe-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<SetActionRecipe-response> is deprecated: use re_srvs-srv:SetActionRecipe-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetActionRecipe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetActionRecipe-response>) ostream)
  "Serializes a message object of type '<SetActionRecipe-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetActionRecipe-response>) istream)
  "Deserializes a message object of type '<SetActionRecipe-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetActionRecipe-response>)))
  "Returns string type for a service object of type '<SetActionRecipe-response>"
  "re_srvs/SetActionRecipeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetActionRecipe-response)))
  "Returns string type for a service object of type 'SetActionRecipe-response"
  "re_srvs/SetActionRecipeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetActionRecipe-response>)))
  "Returns md5sum for a message object of type '<SetActionRecipe-response>"
  "f36f93785e5f7f843ef366910639a201")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetActionRecipe-response)))
  "Returns md5sum for a message object of type 'SetActionRecipe-response"
  "f36f93785e5f7f843ef366910639a201")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetActionRecipe-response>)))
  "Returns full string definition for message of type '<SetActionRecipe-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetActionRecipe-response)))
  "Returns full string definition for message of type 'SetActionRecipe-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetActionRecipe-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetActionRecipe-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetActionRecipe-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetActionRecipe)))
  'SetActionRecipe-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetActionRecipe)))
  'SetActionRecipe-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetActionRecipe)))
  "Returns string type for a service object of type '<SetActionRecipe>"
  "re_srvs/SetActionRecipe")