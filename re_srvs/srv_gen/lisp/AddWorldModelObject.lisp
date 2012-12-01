; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude AddWorldModelObject-request.msg.html

(cl:defclass <AddWorldModelObject-request> (roslisp-msg-protocol:ros-message)
  ((class_label
    :reader class_label
    :initarg :class_label
    :type cl:string
    :initform "")
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass AddWorldModelObject-request (<AddWorldModelObject-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddWorldModelObject-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddWorldModelObject-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<AddWorldModelObject-request> is deprecated: use re_srvs-srv:AddWorldModelObject-request instead.")))

(cl:ensure-generic-function 'class_label-val :lambda-list '(m))
(cl:defmethod class_label-val ((m <AddWorldModelObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:class_label-val is deprecated.  Use re_srvs-srv:class_label instead.")
  (class_label m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <AddWorldModelObject-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:pose-val is deprecated.  Use re_srvs-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddWorldModelObject-request>) ostream)
  "Serializes a message object of type '<AddWorldModelObject-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'class_label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'class_label))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddWorldModelObject-request>) istream)
  "Deserializes a message object of type '<AddWorldModelObject-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'class_label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'class_label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddWorldModelObject-request>)))
  "Returns string type for a service object of type '<AddWorldModelObject-request>"
  "re_srvs/AddWorldModelObjectRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddWorldModelObject-request)))
  "Returns string type for a service object of type 'AddWorldModelObject-request"
  "re_srvs/AddWorldModelObjectRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddWorldModelObject-request>)))
  "Returns md5sum for a message object of type '<AddWorldModelObject-request>"
  "411afd1e2f89d68762aa3bff8f0f33f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddWorldModelObject-request)))
  "Returns md5sum for a message object of type 'AddWorldModelObject-request"
  "411afd1e2f89d68762aa3bff8f0f33f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddWorldModelObject-request>)))
  "Returns full string definition for message of type '<AddWorldModelObject-request>"
  (cl:format cl:nil "string class_label~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddWorldModelObject-request)))
  "Returns full string definition for message of type 'AddWorldModelObject-request"
  (cl:format cl:nil "string class_label~%geometry_msgs/PoseStamped pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddWorldModelObject-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'class_label))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddWorldModelObject-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddWorldModelObject-request
    (cl:cons ':class_label (class_label msg))
    (cl:cons ':pose (pose msg))
))
;//! \htmlinclude AddWorldModelObject-response.msg.html

(cl:defclass <AddWorldModelObject-response> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:integer
    :initform 0))
)

(cl:defclass AddWorldModelObject-response (<AddWorldModelObject-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddWorldModelObject-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddWorldModelObject-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<AddWorldModelObject-response> is deprecated: use re_srvs-srv:AddWorldModelObject-response instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <AddWorldModelObject-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:ID-val is deprecated.  Use re_srvs-srv:ID instead.")
  (ID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddWorldModelObject-response>) ostream)
  "Serializes a message object of type '<AddWorldModelObject-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ID)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddWorldModelObject-response>) istream)
  "Deserializes a message object of type '<AddWorldModelObject-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ID)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddWorldModelObject-response>)))
  "Returns string type for a service object of type '<AddWorldModelObject-response>"
  "re_srvs/AddWorldModelObjectResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddWorldModelObject-response)))
  "Returns string type for a service object of type 'AddWorldModelObject-response"
  "re_srvs/AddWorldModelObjectResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddWorldModelObject-response>)))
  "Returns md5sum for a message object of type '<AddWorldModelObject-response>"
  "411afd1e2f89d68762aa3bff8f0f33f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddWorldModelObject-response)))
  "Returns md5sum for a message object of type 'AddWorldModelObject-response"
  "411afd1e2f89d68762aa3bff8f0f33f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddWorldModelObject-response>)))
  "Returns full string definition for message of type '<AddWorldModelObject-response>"
  (cl:format cl:nil "uint32 ID~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddWorldModelObject-response)))
  "Returns full string definition for message of type 'AddWorldModelObject-response"
  (cl:format cl:nil "uint32 ID~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddWorldModelObject-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddWorldModelObject-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddWorldModelObject-response
    (cl:cons ':ID (ID msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddWorldModelObject)))
  'AddWorldModelObject-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddWorldModelObject)))
  'AddWorldModelObject-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddWorldModelObject)))
  "Returns string type for a service object of type '<AddWorldModelObject>"
  "re_srvs/AddWorldModelObject")