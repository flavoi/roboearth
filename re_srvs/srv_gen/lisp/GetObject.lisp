; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude GetObject-request.msg.html

(cl:defclass <GetObject-request> (roslisp-msg-protocol:ros-message)
  ((objectUID
    :reader objectUID
    :initarg :objectUID
    :type cl:string
    :initform ""))
)

(cl:defclass GetObject-request (<GetObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<GetObject-request> is deprecated: use re_srvs-srv:GetObject-request instead.")))

(cl:ensure-generic-function 'objectUID-val :lambda-list '(m))
(cl:defmethod objectUID-val ((m <GetObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:objectUID-val is deprecated.  Use re_srvs-srv:objectUID instead.")
  (objectUID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetObject-request>) ostream)
  "Serializes a message object of type '<GetObject-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'objectUID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'objectUID))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetObject-request>) istream)
  "Deserializes a message object of type '<GetObject-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'objectUID) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'objectUID) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetObject-request>)))
  "Returns string type for a service object of type '<GetObject-request>"
  "re_srvs/GetObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObject-request)))
  "Returns string type for a service object of type 'GetObject-request"
  "re_srvs/GetObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetObject-request>)))
  "Returns md5sum for a message object of type '<GetObject-request>"
  "6fb904ebd7c4b2123383460b1386ddc5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetObject-request)))
  "Returns md5sum for a message object of type 'GetObject-request"
  "6fb904ebd7c4b2123383460b1386ddc5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetObject-request>)))
  "Returns full string definition for message of type '<GetObject-request>"
  (cl:format cl:nil "string objectUID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetObject-request)))
  "Returns full string definition for message of type 'GetObject-request"
  (cl:format cl:nil "string objectUID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetObject-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'objectUID))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetObject-request
    (cl:cons ':objectUID (objectUID msg))
))
;//! \htmlinclude GetObject-response.msg.html

(cl:defclass <GetObject-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (object
    :reader object
    :initarg :object
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

(cl:defclass GetObject-response (<GetObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<GetObject-response> is deprecated: use re_srvs-srv:GetObject-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <GetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:object-val is deprecated.  Use re_srvs-srv:object instead.")
  (object m))

(cl:ensure-generic-function 'filenames-val :lambda-list '(m))
(cl:defmethod filenames-val ((m <GetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:filenames-val is deprecated.  Use re_srvs-srv:filenames instead.")
  (filenames m))

(cl:ensure-generic-function 'fileURLs-val :lambda-list '(m))
(cl:defmethod fileURLs-val ((m <GetObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:fileURLs-val is deprecated.  Use re_srvs-srv:fileURLs instead.")
  (fileURLs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetObject-response>) ostream)
  "Serializes a message object of type '<GetObject-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetObject-response>) istream)
  "Deserializes a message object of type '<GetObject-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetObject-response>)))
  "Returns string type for a service object of type '<GetObject-response>"
  "re_srvs/GetObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObject-response)))
  "Returns string type for a service object of type 'GetObject-response"
  "re_srvs/GetObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetObject-response>)))
  "Returns md5sum for a message object of type '<GetObject-response>"
  "6fb904ebd7c4b2123383460b1386ddc5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetObject-response)))
  "Returns md5sum for a message object of type 'GetObject-response"
  "6fb904ebd7c4b2123383460b1386ddc5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetObject-response>)))
  "Returns full string definition for message of type '<GetObject-response>"
  (cl:format cl:nil "bool success~%string object~%string[] filenames~%string[] fileURLs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetObject-response)))
  "Returns full string definition for message of type 'GetObject-response"
  (cl:format cl:nil "bool success~%string object~%string[] filenames~%string[] fileURLs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetObject-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'object))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'filenames) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'fileURLs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetObject-response
    (cl:cons ':success (success msg))
    (cl:cons ':object (object msg))
    (cl:cons ':filenames (filenames msg))
    (cl:cons ':fileURLs (fileURLs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetObject)))
  'GetObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetObject)))
  'GetObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetObject)))
  "Returns string type for a service object of type '<GetObject>"
  "re_srvs/GetObject")