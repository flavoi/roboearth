; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude GetActionRecipe-request.msg.html

(cl:defclass <GetActionRecipe-request> (roslisp-msg-protocol:ros-message)
  ((recipeUID
    :reader recipeUID
    :initarg :recipeUID
    :type cl:string
    :initform ""))
)

(cl:defclass GetActionRecipe-request (<GetActionRecipe-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetActionRecipe-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetActionRecipe-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<GetActionRecipe-request> is deprecated: use re_srvs-srv:GetActionRecipe-request instead.")))

(cl:ensure-generic-function 'recipeUID-val :lambda-list '(m))
(cl:defmethod recipeUID-val ((m <GetActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:recipeUID-val is deprecated.  Use re_srvs-srv:recipeUID instead.")
  (recipeUID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetActionRecipe-request>) ostream)
  "Serializes a message object of type '<GetActionRecipe-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'recipeUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'recipeUID))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetActionRecipe-request>) istream)
  "Deserializes a message object of type '<GetActionRecipe-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'recipeUID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'recipeUID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetActionRecipe-request>)))
  "Returns string type for a service object of type '<GetActionRecipe-request>"
  "re_srvs/GetActionRecipeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetActionRecipe-request)))
  "Returns string type for a service object of type 'GetActionRecipe-request"
  "re_srvs/GetActionRecipeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetActionRecipe-request>)))
  "Returns md5sum for a message object of type '<GetActionRecipe-request>"
  "aa14c50affb322948b40216eb0e708a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetActionRecipe-request)))
  "Returns md5sum for a message object of type 'GetActionRecipe-request"
  "aa14c50affb322948b40216eb0e708a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetActionRecipe-request>)))
  "Returns full string definition for message of type '<GetActionRecipe-request>"
  (cl:format cl:nil "string recipeUID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetActionRecipe-request)))
  "Returns full string definition for message of type 'GetActionRecipe-request"
  (cl:format cl:nil "string recipeUID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetActionRecipe-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'recipeUID))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetActionRecipe-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetActionRecipe-request
    (cl:cons ':recipeUID (recipeUID msg))
))
;//! \htmlinclude GetActionRecipe-response.msg.html

(cl:defclass <GetActionRecipe-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (recipe
    :reader recipe
    :initarg :recipe
    :type cl:string
    :initform ""))
)

(cl:defclass GetActionRecipe-response (<GetActionRecipe-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetActionRecipe-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetActionRecipe-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<GetActionRecipe-response> is deprecated: use re_srvs-srv:GetActionRecipe-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetActionRecipe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'recipe-val :lambda-list '(m))
(cl:defmethod recipe-val ((m <GetActionRecipe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:recipe-val is deprecated.  Use re_srvs-srv:recipe instead.")
  (recipe m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetActionRecipe-response>) ostream)
  "Serializes a message object of type '<GetActionRecipe-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'recipe))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'recipe))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetActionRecipe-response>) istream)
  "Deserializes a message object of type '<GetActionRecipe-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'recipe) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'recipe) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetActionRecipe-response>)))
  "Returns string type for a service object of type '<GetActionRecipe-response>"
  "re_srvs/GetActionRecipeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetActionRecipe-response)))
  "Returns string type for a service object of type 'GetActionRecipe-response"
  "re_srvs/GetActionRecipeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetActionRecipe-response>)))
  "Returns md5sum for a message object of type '<GetActionRecipe-response>"
  "aa14c50affb322948b40216eb0e708a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetActionRecipe-response)))
  "Returns md5sum for a message object of type 'GetActionRecipe-response"
  "aa14c50affb322948b40216eb0e708a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetActionRecipe-response>)))
  "Returns full string definition for message of type '<GetActionRecipe-response>"
  (cl:format cl:nil "bool success~%string recipe~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetActionRecipe-response)))
  "Returns full string definition for message of type 'GetActionRecipe-response"
  (cl:format cl:nil "bool success~%string recipe~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetActionRecipe-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'recipe))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetActionRecipe-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetActionRecipe-response
    (cl:cons ':success (success msg))
    (cl:cons ':recipe (recipe msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetActionRecipe)))
  'GetActionRecipe-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetActionRecipe)))
  'GetActionRecipe-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetActionRecipe)))
  "Returns string type for a service object of type '<GetActionRecipe>"
  "re_srvs/GetActionRecipe")