; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude DelActionRecipe-request.msg.html

(cl:defclass <DelActionRecipe-request> (roslisp-msg-protocol:ros-message)
  ((recipeUID
    :reader recipeUID
    :initarg :recipeUID
    :type cl:string
    :initform "")
   (apiKey
    :reader apiKey
    :initarg :apiKey
    :type cl:string
    :initform ""))
)

(cl:defclass DelActionRecipe-request (<DelActionRecipe-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DelActionRecipe-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DelActionRecipe-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DelActionRecipe-request> is deprecated: use re_srvs-srv:DelActionRecipe-request instead.")))

(cl:ensure-generic-function 'recipeUID-val :lambda-list '(m))
(cl:defmethod recipeUID-val ((m <DelActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:recipeUID-val is deprecated.  Use re_srvs-srv:recipeUID instead.")
  (recipeUID m))

(cl:ensure-generic-function 'apiKey-val :lambda-list '(m))
(cl:defmethod apiKey-val ((m <DelActionRecipe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:apiKey-val is deprecated.  Use re_srvs-srv:apiKey instead.")
  (apiKey m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DelActionRecipe-request>) ostream)
  "Serializes a message object of type '<DelActionRecipe-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'recipeUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'recipeUID))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'apiKey))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'apiKey))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DelActionRecipe-request>) istream)
  "Deserializes a message object of type '<DelActionRecipe-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'recipeUID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'recipeUID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DelActionRecipe-request>)))
  "Returns string type for a service object of type '<DelActionRecipe-request>"
  "re_srvs/DelActionRecipeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelActionRecipe-request)))
  "Returns string type for a service object of type 'DelActionRecipe-request"
  "re_srvs/DelActionRecipeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DelActionRecipe-request>)))
  "Returns md5sum for a message object of type '<DelActionRecipe-request>"
  "0e76089db06866f69154779bbe023e21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DelActionRecipe-request)))
  "Returns md5sum for a message object of type 'DelActionRecipe-request"
  "0e76089db06866f69154779bbe023e21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DelActionRecipe-request>)))
  "Returns full string definition for message of type '<DelActionRecipe-request>"
  (cl:format cl:nil "string recipeUID~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DelActionRecipe-request)))
  "Returns full string definition for message of type 'DelActionRecipe-request"
  (cl:format cl:nil "string recipeUID~%string apiKey~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DelActionRecipe-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'recipeUID))
     4 (cl:length (cl:slot-value msg 'apiKey))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DelActionRecipe-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DelActionRecipe-request
    (cl:cons ':recipeUID (recipeUID msg))
    (cl:cons ':apiKey (apiKey msg))
))
;//! \htmlinclude DelActionRecipe-response.msg.html

(cl:defclass <DelActionRecipe-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DelActionRecipe-response (<DelActionRecipe-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DelActionRecipe-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DelActionRecipe-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DelActionRecipe-response> is deprecated: use re_srvs-srv:DelActionRecipe-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DelActionRecipe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DelActionRecipe-response>) ostream)
  "Serializes a message object of type '<DelActionRecipe-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DelActionRecipe-response>) istream)
  "Deserializes a message object of type '<DelActionRecipe-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DelActionRecipe-response>)))
  "Returns string type for a service object of type '<DelActionRecipe-response>"
  "re_srvs/DelActionRecipeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelActionRecipe-response)))
  "Returns string type for a service object of type 'DelActionRecipe-response"
  "re_srvs/DelActionRecipeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DelActionRecipe-response>)))
  "Returns md5sum for a message object of type '<DelActionRecipe-response>"
  "0e76089db06866f69154779bbe023e21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DelActionRecipe-response)))
  "Returns md5sum for a message object of type 'DelActionRecipe-response"
  "0e76089db06866f69154779bbe023e21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DelActionRecipe-response>)))
  "Returns full string definition for message of type '<DelActionRecipe-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DelActionRecipe-response)))
  "Returns full string definition for message of type 'DelActionRecipe-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DelActionRecipe-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DelActionRecipe-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DelActionRecipe-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DelActionRecipe)))
  'DelActionRecipe-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DelActionRecipe)))
  'DelActionRecipe-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DelActionRecipe)))
  "Returns string type for a service object of type '<DelActionRecipe>"
  "re_srvs/DelActionRecipe")