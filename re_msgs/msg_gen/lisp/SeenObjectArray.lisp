; Auto-generated. Do not edit!


(cl:in-package re_msgs-msg)


;//! \htmlinclude SeenObjectArray.msg.html

(cl:defclass <SeenObjectArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (object
    :reader object
    :initarg :object
    :type (cl:vector re_msgs-msg:SeenObject)
   :initform (cl:make-array 0 :element-type 're_msgs-msg:SeenObject :initial-element (cl:make-instance 're_msgs-msg:SeenObject))))
)

(cl:defclass SeenObjectArray (<SeenObjectArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SeenObjectArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SeenObjectArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_msgs-msg:<SeenObjectArray> is deprecated: use re_msgs-msg:SeenObjectArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SeenObjectArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_msgs-msg:header-val is deprecated.  Use re_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <SeenObjectArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_msgs-msg:object-val is deprecated.  Use re_msgs-msg:object instead.")
  (object m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SeenObjectArray>) ostream)
  "Serializes a message object of type '<SeenObjectArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'object))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'object))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SeenObjectArray>) istream)
  "Deserializes a message object of type '<SeenObjectArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'object) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'object)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 're_msgs-msg:SeenObject))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SeenObjectArray>)))
  "Returns string type for a message object of type '<SeenObjectArray>"
  "re_msgs/SeenObjectArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SeenObjectArray)))
  "Returns string type for a message object of type 'SeenObjectArray"
  "re_msgs/SeenObjectArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SeenObjectArray>)))
  "Returns md5sum for a message object of type '<SeenObjectArray>"
  "db06ee56fb2cfb4568deee7165cba35f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SeenObjectArray)))
  "Returns md5sum for a message object of type 'SeenObjectArray"
  "db06ee56fb2cfb4568deee7165cba35f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SeenObjectArray>)))
  "Returns full string definition for message of type '<SeenObjectArray>"
  (cl:format cl:nil "#This represents a list of objects~%Header header~%SeenObject[] object~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: re_msgs/SeenObject~%#This represents a Object~%time stamp~%string name~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SeenObjectArray)))
  "Returns full string definition for message of type 'SeenObjectArray"
  (cl:format cl:nil "#This represents a list of objects~%Header header~%SeenObject[] object~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: re_msgs/SeenObject~%#This represents a Object~%time stamp~%string name~%geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SeenObjectArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'object) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SeenObjectArray>))
  "Converts a ROS message object to a list"
  (cl:list 'SeenObjectArray
    (cl:cons ':header (header msg))
    (cl:cons ':object (object msg))
))
