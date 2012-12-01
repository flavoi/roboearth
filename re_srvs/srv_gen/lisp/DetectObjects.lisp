; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude DetectObjects-request.msg.html

(cl:defclass <DetectObjects-request> (roslisp-msg-protocol:ros-message)
  ((uids
    :reader uids
    :initarg :uids
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass DetectObjects-request (<DetectObjects-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectObjects-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectObjects-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DetectObjects-request> is deprecated: use re_srvs-srv:DetectObjects-request instead.")))

(cl:ensure-generic-function 'uids-val :lambda-list '(m))
(cl:defmethod uids-val ((m <DetectObjects-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:uids-val is deprecated.  Use re_srvs-srv:uids instead.")
  (uids m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectObjects-request>) ostream)
  "Serializes a message object of type '<DetectObjects-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectObjects-request>) istream)
  "Deserializes a message object of type '<DetectObjects-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectObjects-request>)))
  "Returns string type for a service object of type '<DetectObjects-request>"
  "re_srvs/DetectObjectsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectObjects-request)))
  "Returns string type for a service object of type 'DetectObjects-request"
  "re_srvs/DetectObjectsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectObjects-request>)))
  "Returns md5sum for a message object of type '<DetectObjects-request>"
  "8521e6e6a78bb6612fbdd6cf68737d7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectObjects-request)))
  "Returns md5sum for a message object of type 'DetectObjects-request"
  "8521e6e6a78bb6612fbdd6cf68737d7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectObjects-request>)))
  "Returns full string definition for message of type '<DetectObjects-request>"
  (cl:format cl:nil "~%~%string[] uids~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectObjects-request)))
  "Returns full string definition for message of type 'DetectObjects-request"
  (cl:format cl:nil "~%~%string[] uids~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectObjects-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'uids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectObjects-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectObjects-request
    (cl:cons ':uids (uids msg))
))
;//! \htmlinclude DetectObjects-response.msg.html

(cl:defclass <DetectObjects-response> (roslisp-msg-protocol:ros-message)
  ((detectableUIDs
    :reader detectableUIDs
    :initarg :detectableUIDs
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass DetectObjects-response (<DetectObjects-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectObjects-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectObjects-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<DetectObjects-response> is deprecated: use re_srvs-srv:DetectObjects-response instead.")))

(cl:ensure-generic-function 'detectableUIDs-val :lambda-list '(m))
(cl:defmethod detectableUIDs-val ((m <DetectObjects-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:detectableUIDs-val is deprecated.  Use re_srvs-srv:detectableUIDs instead.")
  (detectableUIDs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectObjects-response>) ostream)
  "Serializes a message object of type '<DetectObjects-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detectableUIDs))))
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
   (cl:slot-value msg 'detectableUIDs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectObjects-response>) istream)
  "Deserializes a message object of type '<DetectObjects-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detectableUIDs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detectableUIDs)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectObjects-response>)))
  "Returns string type for a service object of type '<DetectObjects-response>"
  "re_srvs/DetectObjectsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectObjects-response)))
  "Returns string type for a service object of type 'DetectObjects-response"
  "re_srvs/DetectObjectsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectObjects-response>)))
  "Returns md5sum for a message object of type '<DetectObjects-response>"
  "8521e6e6a78bb6612fbdd6cf68737d7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectObjects-response)))
  "Returns md5sum for a message object of type 'DetectObjects-response"
  "8521e6e6a78bb6612fbdd6cf68737d7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectObjects-response>)))
  "Returns full string definition for message of type '<DetectObjects-response>"
  (cl:format cl:nil "string[] detectableUIDs~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectObjects-response)))
  "Returns full string definition for message of type 'DetectObjects-response"
  (cl:format cl:nil "string[] detectableUIDs~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectObjects-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detectableUIDs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectObjects-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectObjects-response
    (cl:cons ':detectableUIDs (detectableUIDs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DetectObjects)))
  'DetectObjects-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DetectObjects)))
  'DetectObjects-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectObjects)))
  "Returns string type for a service object of type '<DetectObjects>"
  "re_srvs/DetectObjects")