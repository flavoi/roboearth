; Auto-generated. Do not edit!


(cl:in-package re_msgs-msg)


;//! \htmlinclude StringArray.msg.html

(cl:defclass <StringArray> (roslisp-msg-protocol:ros-message)
  ((list
    :reader list
    :initarg :list
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass StringArray (<StringArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StringArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StringArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_msgs-msg:<StringArray> is deprecated: use re_msgs-msg:StringArray instead.")))

(cl:ensure-generic-function 'list-val :lambda-list '(m))
(cl:defmethod list-val ((m <StringArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_msgs-msg:list-val is deprecated.  Use re_msgs-msg:list instead.")
  (list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StringArray>) ostream)
  "Serializes a message object of type '<StringArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'list))))
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
   (cl:slot-value msg 'list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StringArray>) istream)
  "Deserializes a message object of type '<StringArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'list)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StringArray>)))
  "Returns string type for a message object of type '<StringArray>"
  "re_msgs/StringArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StringArray)))
  "Returns string type for a message object of type 'StringArray"
  "re_msgs/StringArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StringArray>)))
  "Returns md5sum for a message object of type '<StringArray>"
  "8e09fa9aad51508cfdcab3b64f048ce0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StringArray)))
  "Returns md5sum for a message object of type 'StringArray"
  "8e09fa9aad51508cfdcab3b64f048ce0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StringArray>)))
  "Returns full string definition for message of type '<StringArray>"
  (cl:format cl:nil "# A StringArray message contains an array of strings. This is used by other ~%# message/service declarations in order to create 2-dimensional string ~%# arrays with different lengths for one dimension (StringArray[])~%string[] list   # array of strings~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StringArray)))
  "Returns full string definition for message of type 'StringArray"
  (cl:format cl:nil "# A StringArray message contains an array of strings. This is used by other ~%# message/service declarations in order to create 2-dimensional string ~%# arrays with different lengths for one dimension (StringArray[])~%string[] list   # array of strings~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StringArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StringArray>))
  "Converts a ROS message object to a list"
  (cl:list 'StringArray
    (cl:cons ':list (list msg))
))
