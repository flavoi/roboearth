; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude SearchObjects-request.msg.html

(cl:defclass <SearchObjects-request> (roslisp-msg-protocol:ros-message)
  ((searchID
    :reader searchID
    :initarg :searchID
    :type cl:string
    :initform ""))
)

(cl:defclass SearchObjects-request (<SearchObjects-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SearchObjects-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SearchObjects-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<SearchObjects-request> is deprecated: use re_srvs-srv:SearchObjects-request instead.")))

(cl:ensure-generic-function 'searchID-val :lambda-list '(m))
(cl:defmethod searchID-val ((m <SearchObjects-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:searchID-val is deprecated.  Use re_srvs-srv:searchID instead.")
  (searchID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SearchObjects-request>) ostream)
  "Serializes a message object of type '<SearchObjects-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'searchID))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'searchID))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SearchObjects-request>) istream)
  "Deserializes a message object of type '<SearchObjects-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SearchObjects-request>)))
  "Returns string type for a service object of type '<SearchObjects-request>"
  "re_srvs/SearchObjectsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchObjects-request)))
  "Returns string type for a service object of type 'SearchObjects-request"
  "re_srvs/SearchObjectsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SearchObjects-request>)))
  "Returns md5sum for a message object of type '<SearchObjects-request>"
  "2dfe23d35da7ced197e999d2dd7125f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SearchObjects-request)))
  "Returns md5sum for a message object of type 'SearchObjects-request"
  "2dfe23d35da7ced197e999d2dd7125f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SearchObjects-request>)))
  "Returns full string definition for message of type '<SearchObjects-request>"
  (cl:format cl:nil "string searchID~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SearchObjects-request)))
  "Returns full string definition for message of type 'SearchObjects-request"
  (cl:format cl:nil "string searchID~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SearchObjects-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'searchID))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SearchObjects-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SearchObjects-request
    (cl:cons ':searchID (searchID msg))
))
;//! \htmlinclude SearchObjects-response.msg.html

(cl:defclass <SearchObjects-response> (roslisp-msg-protocol:ros-message)
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
   (objects
    :reader objects
    :initarg :objects
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (filenames
    :reader filenames
    :initarg :filenames
    :type (cl:vector re_msgs-msg:StringArray)
   :initform (cl:make-array 0 :element-type 're_msgs-msg:StringArray :initial-element (cl:make-instance 're_msgs-msg:StringArray)))
   (fileURLs
    :reader fileURLs
    :initarg :fileURLs
    :type (cl:vector re_msgs-msg:StringArray)
   :initform (cl:make-array 0 :element-type 're_msgs-msg:StringArray :initial-element (cl:make-instance 're_msgs-msg:StringArray))))
)

(cl:defclass SearchObjects-response (<SearchObjects-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SearchObjects-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SearchObjects-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<SearchObjects-response> is deprecated: use re_srvs-srv:SearchObjects-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SearchObjects-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:success-val is deprecated.  Use re_srvs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'uids-val :lambda-list '(m))
(cl:defmethod uids-val ((m <SearchObjects-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:uids-val is deprecated.  Use re_srvs-srv:uids instead.")
  (uids m))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <SearchObjects-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:objects-val is deprecated.  Use re_srvs-srv:objects instead.")
  (objects m))

(cl:ensure-generic-function 'filenames-val :lambda-list '(m))
(cl:defmethod filenames-val ((m <SearchObjects-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:filenames-val is deprecated.  Use re_srvs-srv:filenames instead.")
  (filenames m))

(cl:ensure-generic-function 'fileURLs-val :lambda-list '(m))
(cl:defmethod fileURLs-val ((m <SearchObjects-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:fileURLs-val is deprecated.  Use re_srvs-srv:fileURLs instead.")
  (fileURLs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SearchObjects-response>) ostream)
  "Serializes a message object of type '<SearchObjects-response>"
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
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
   (cl:slot-value msg 'objects))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'filenames))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'filenames))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'fileURLs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'fileURLs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SearchObjects-response>) istream)
  "Deserializes a message object of type '<SearchObjects-response>"
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
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
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
  (cl:setf (cl:slot-value msg 'filenames) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'filenames)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 're_msgs-msg:StringArray))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'fileURLs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'fileURLs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 're_msgs-msg:StringArray))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SearchObjects-response>)))
  "Returns string type for a service object of type '<SearchObjects-response>"
  "re_srvs/SearchObjectsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchObjects-response)))
  "Returns string type for a service object of type 'SearchObjects-response"
  "re_srvs/SearchObjectsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SearchObjects-response>)))
  "Returns md5sum for a message object of type '<SearchObjects-response>"
  "2dfe23d35da7ced197e999d2dd7125f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SearchObjects-response)))
  "Returns md5sum for a message object of type 'SearchObjects-response"
  "2dfe23d35da7ced197e999d2dd7125f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SearchObjects-response>)))
  "Returns full string definition for message of type '<SearchObjects-response>"
  (cl:format cl:nil "bool success~%string[] uids~%string[] objects~%re_msgs/StringArray[] filenames~%re_msgs/StringArray[] fileURLs~%~%~%================================================================================~%MSG: re_msgs/StringArray~%# A StringArray message contains an array of strings. This is used by other ~%# message/service declarations in order to create 2-dimensional string ~%# arrays with different lengths for one dimension (StringArray[])~%string[] list   # array of strings~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SearchObjects-response)))
  "Returns full string definition for message of type 'SearchObjects-response"
  (cl:format cl:nil "bool success~%string[] uids~%string[] objects~%re_msgs/StringArray[] filenames~%re_msgs/StringArray[] fileURLs~%~%~%================================================================================~%MSG: re_msgs/StringArray~%# A StringArray message contains an array of strings. This is used by other ~%# message/service declarations in order to create 2-dimensional string ~%# arrays with different lengths for one dimension (StringArray[])~%string[] list   # array of strings~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SearchObjects-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'uids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'filenames) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'fileURLs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SearchObjects-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SearchObjects-response
    (cl:cons ':success (success msg))
    (cl:cons ':uids (uids msg))
    (cl:cons ':objects (objects msg))
    (cl:cons ':filenames (filenames msg))
    (cl:cons ':fileURLs (fileURLs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SearchObjects)))
  'SearchObjects-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SearchObjects)))
  'SearchObjects-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchObjects)))
  "Returns string type for a service object of type '<SearchObjects>"
  "re_srvs/SearchObjects")