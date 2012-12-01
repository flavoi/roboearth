; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude GetEnvironment-request.msg.html

(cl:defclass <GetEnvironment-request> (roslisp-msg-protocol:ros-message)
  ((environmentUID
    :reader environmentUID
    :initarg :environmentUID
    :type cl:string
    :initform ""))
)

(cl:defclass GetEnvironment-request (<GetEnvironment-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetEnvironment-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetEnvironment-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<GetEnvironment-request> is deprecated: use re_srvs-srv:GetEnvironment-request instead.")))

(cl:ensure-generic-function 'environmentUID-val :lambda-list '(m))
(cl:defmethod environmentUID-val ((m <GetEnvironment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:environmentUID-val is deprecated.  Use re_srvs-srv:environmentUID instead.")
  (environmentUID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetEnvironment-request>) ostream)
  "Serializes a message object of type '<GetEnvironment-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'environmentUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'environmentUID))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetEnvironment-request>) istream)
  "Deserializes a message object of type '<GetEnvironment-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'environmentUID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'environmentUID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetEnvironment-request>)))
  "Returns string type for a service object of type '<GetEnvironment-request>"
  "re_srvs/GetEnvironmentRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEnvironment-request)))
  "Returns string type for a service object of type 'GetEnvironment-request"
  "re_srvs/GetEnvironmentRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetEnvironment-request>)))
  "Returns md5sum for a message object of type '<GetEnvironment-request>"
  "c017f96c32c005323930741129f3a27d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetEnvironment-request)))
  "Returns md5sum for a message object of type 'GetEnvironment-request"
  "c017f96c32c005323930741129f3a27d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetEnvironment-request>)))
  "Returns full string definition for message of type '<GetEnvironment-request>"
  (cl:format cl:nil "string environmentUID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetEnvironment-request)))
  "Returns full string definition for message of type 'GetEnvironment-request"
  (cl:format cl:nil "string environmentUID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetEnvironment-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'environmentUID))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetEnvironment-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetEnvironment-request
    (cl:cons ':environmentUID (environmentUID msg))
))
;//! \htmlinclude GetEnvironment-response.msg.html

(cl:defclass <GetEnvironment-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (environment
    :reader environment
    :initarg :environment
    :type cl:string
    :initform "")
   (filenames
    :reader filenames
    :initarg :filenames
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (fileURLs
    :reader fileURLs
    :initarg :fileURLs
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass GetEnvironment-response (<GetEnvironment-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetEnvironment-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetEnvironment-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<GetEnvironment-response> is deprecated: use re_srvs-srv:GetEnvironment-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetEnvironment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'environment-val :lambda-list '(m))
(cl:defmethod environment-val ((m <GetEnvironment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:environment-val is deprecated.  Use re_srvs-srv:environment instead.")
  (environment m))

(cl:ensure-generic-function 'filenames-val :lambda-list '(m))
(cl:defmethod filenames-val ((m <GetEnvironment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:filenames-val is deprecated.  Use re_srvs-srv:filenames instead.")
  (filenames m))

(cl:ensure-generic-function 'fileURLs-val :lambda-list '(m))
(cl:defmethod fileURLs-val ((m <GetEnvironment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:fileURLs-val is deprecated.  Use re_srvs-srv:fileURLs instead.")
  (fileURLs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetEnvironment-response>) ostream)
  "Serializes a message object of type '<GetEnvironment-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'environment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'environment))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'filenames))))
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
   (cl:slot-value msg 'filenames))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'fileURLs))))
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
   (cl:slot-value msg 'fileURLs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetEnvironment-response>) istream)
  "Deserializes a message object of type '<GetEnvironment-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'environment) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'environment) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'filenames) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'filenames)))
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
  (cl:setf (cl:slot-value msg 'fileURLs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'fileURLs)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetEnvironment-response>)))
  "Returns string type for a service object of type '<GetEnvironment-response>"
  "re_srvs/GetEnvironmentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEnvironment-response)))
  "Returns string type for a service object of type 'GetEnvironment-response"
  "re_srvs/GetEnvironmentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetEnvironment-response>)))
  "Returns md5sum for a message object of type '<GetEnvironment-response>"
  "c017f96c32c005323930741129f3a27d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetEnvironment-response)))
  "Returns md5sum for a message object of type 'GetEnvironment-response"
  "c017f96c32c005323930741129f3a27d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetEnvironment-response>)))
  "Returns full string definition for message of type '<GetEnvironment-response>"
  (cl:format cl:nil "bool success~%string environment~%string[] filenames~%string[] fileURLs~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetEnvironment-response)))
  "Returns full string definition for message of type 'GetEnvironment-response"
  (cl:format cl:nil "bool success~%string environment~%string[] filenames~%string[] fileURLs~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetEnvironment-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'environment))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'filenames) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'fileURLs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetEnvironment-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetEnvironment-response
    (cl:cons ':success (success msg))
    (cl:cons ':environment (environment msg))
    (cl:cons ':filenames (filenames msg))
    (cl:cons ':fileURLs (fileURLs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetEnvironment)))
  'GetEnvironment-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetEnvironment)))
  'GetEnvironment-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEnvironment)))
  "Returns string type for a service object of type '<GetEnvironment>"
  "re_srvs/GetEnvironment")