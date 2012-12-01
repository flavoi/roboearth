; Auto-generated. Do not edit!


(cl:in-package re_msgs-msg)


;//! \htmlinclude Pose2DStamped.msg.html

(cl:defclass <Pose2DStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D)))
)

(cl:defclass Pose2DStamped (<Pose2DStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pose2DStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pose2DStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_msgs-msg:<Pose2DStamped> is deprecated: use re_msgs-msg:Pose2DStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Pose2DStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_msgs-msg:header-val is deprecated.  Use re_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Pose2DStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_msgs-msg:pose-val is deprecated.  Use re_msgs-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pose2DStamped>) ostream)
  "Serializes a message object of type '<Pose2DStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pose2DStamped>) istream)
  "Deserializes a message object of type '<Pose2DStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pose2DStamped>)))
  "Returns string type for a message object of type '<Pose2DStamped>"
  "re_msgs/Pose2DStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pose2DStamped)))
  "Returns string type for a message object of type 'Pose2DStamped"
  "re_msgs/Pose2DStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pose2DStamped>)))
  "Returns md5sum for a message object of type '<Pose2DStamped>"
  "b5f1e28823201bc5ea7e310fc49d253f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pose2DStamped)))
  "Returns md5sum for a message object of type 'Pose2DStamped"
  "b5f1e28823201bc5ea7e310fc49d253f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pose2DStamped>)))
  "Returns full string definition for message of type '<Pose2DStamped>"
  (cl:format cl:nil "#This represents a Pose2D with reference coordinate frame and timestamp~%Header header~%geometry_msgs/Pose2D pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pose2DStamped)))
  "Returns full string definition for message of type 'Pose2DStamped"
  (cl:format cl:nil "#This represents a Pose2D with reference coordinate frame and timestamp~%Header header~%geometry_msgs/Pose2D pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pose2DStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pose2DStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'Pose2DStamped
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
))
