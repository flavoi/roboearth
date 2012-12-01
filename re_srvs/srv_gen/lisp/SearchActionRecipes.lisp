; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude SearchActionRecipes-request.msg.html

(cl:defclass <SearchActionRecipes-request> (roslisp-msg-protocol:ros-message)
  ((searchID
    :reader searchID
    :initarg :searchID
    :type cl:string
    :initform ""))
)

(cl:defclass SearchActionRecipes-request (<SearchActionRecipes-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SearchActionRecipes-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SearchActionRecipes-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<SearchActionRecipes-request> is deprecated: use re_srvs-srv:SearchActionRecipes-request instead.")))

(cl:ensure-generic-function 'searchID-val :lambda-list '(m))
(cl:defmethod searchID-val ((m <SearchActionRecipes-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:searchID-val is deprecated.  Use re_srvs-srv:searchID instead.")
  (searchID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SearchActionRecipes-request>) ostream)
  "Serializes a message object of type '<SearchActionRecipes-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'searchID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'searchID))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SearchActionRecipes-request>) istream)
  "Deserializes a message object of type '<SearchActionRecipes-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'searchID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'searchID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SearchActionRecipes-request>)))
  "Returns string type for a service object of type '<SearchActionRecipes-request>"
  "re_srvs/SearchActionRecipesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchActionRecipes-request)))
  "Returns string type for a service object of type 'SearchActionRecipes-request"
  "re_srvs/SearchActionRecipesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SearchActionRecipes-request>)))
  "Returns md5sum for a message object of type '<SearchActionRecipes-request>"
  "ab460e156aa3e532e70c8a5b8e2f72e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SearchActionRecipes-request)))
  "Returns md5sum for a message object of type 'SearchActionRecipes-request"
  "ab460e156aa3e532e70c8a5b8e2f72e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SearchActionRecipes-request>)))
  "Returns full string definition for message of type '<SearchActionRecipes-request>"
  (cl:format cl:nil "string searchID~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SearchActionRecipes-request)))
  "Returns full string definition for message of type 'SearchActionRecipes-request"
  (cl:format cl:nil "string searchID~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SearchActionRecipes-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'searchID))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SearchActionRecipes-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SearchActionRecipes-request
    (cl:cons ':searchID (searchID msg))
))
;//! \htmlinclude SearchActionRecipes-response.msg.html

(cl:defclass <SearchActionRecipes-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (uids
    :reader uids
    :initarg :uids
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (recipes
    :reader recipes
    :initarg :recipes
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass SearchActionRecipes-response (<SearchActionRecipes-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SearchActionRecipes-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SearchActionRecipes-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<SearchActionRecipes-response> is deprecated: use re_srvs-srv:SearchActionRecipes-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SearchActionRecipes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'uids-val :lambda-list '(m))
(cl:defmethod uids-val ((m <SearchActionRecipes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:uids-val is deprecated.  Use re_srvs-srv:uids instead.")
  (uids m))

(cl:ensure-generic-function 'recipes-val :lambda-list '(m))
(cl:defmethod recipes-val ((m <SearchActionRecipes-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:recipes-val is deprecated.  Use re_srvs-srv:recipes instead.")
  (recipes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SearchActionRecipes-response>) ostream)
  "Serializes a message object of type '<SearchActionRecipes-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'uids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'uids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'recipes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'recipes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SearchActionRecipes-response>) istream)
  "Deserializes a message object of type '<SearchActionRecipes-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'uids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'uids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'recipes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'recipes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SearchActionRecipes-response>)))
  "Returns string type for a service object of type '<SearchActionRecipes-response>"
  "re_srvs/SearchActionRecipesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchActionRecipes-response)))
  "Returns string type for a service object of type 'SearchActionRecipes-response"
  "re_srvs/SearchActionRecipesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SearchActionRecipes-response>)))
  "Returns md5sum for a message object of type '<SearchActionRecipes-response>"
  "ab460e156aa3e532e70c8a5b8e2f72e7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SearchActionRecipes-response)))
  "Returns md5sum for a message object of type 'SearchActionRecipes-response"
  "ab460e156aa3e532e70c8a5b8e2f72e7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SearchActionRecipes-response>)))
  "Returns full string definition for message of type '<SearchActionRecipes-response>"
  (cl:format cl:nil "bool success~%string[] uids~%string[] recipes~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SearchActionRecipes-response)))
  "Returns full string definition for message of type 'SearchActionRecipes-response"
  (cl:format cl:nil "bool success~%string[] uids~%string[] recipes~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SearchActionRecipes-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'uids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'recipes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SearchActionRecipes-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SearchActionRecipes-response
    (cl:cons ':success (success msg))
    (cl:cons ':uids (uids msg))
    (cl:cons ':recipes (recipes msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SearchActionRecipes)))
  'SearchActionRecipes-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SearchActionRecipes)))
  'SearchActionRecipes-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchActionRecipes)))
  "Returns string type for a service object of type '<SearchActionRecipes>"
  "re_srvs/SearchActionRecipes")