; Auto-generated. Do not edit!


(cl:in-package re_srvs-srv)


;//! \htmlinclude getNextPose-request.msg.html

(cl:defclass <getNextPose-request> (roslisp-msg-protocol:ros-message)
  ((currentPose
    :reader currentPose
    :initarg :currentPose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass getNextPose-request (<getNextPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getNextPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getNextPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<getNextPose-request> is deprecated: use re_srvs-srv:getNextPose-request instead.")))

(cl:ensure-generic-function 'currentPose-val :lambda-list '(m))
(cl:defmethod currentPose-val ((m <getNextPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:currentPose-val is deprecated.  Use re_srvs-srv:currentPose instead.")
  (currentPose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getNextPose-request>) ostream)
  "Serializes a message object of type '<getNextPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'currentPose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getNextPose-request>) istream)
  "Deserializes a message object of type '<getNextPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'currentPose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getNextPose-request>)))
  "Returns string type for a service object of type '<getNextPose-request>"
  "re_srvs/getNextPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getNextPose-request)))
  "Returns string type for a service object of type 'getNextPose-request"
  "re_srvs/getNextPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getNextPose-request>)))
  "Returns md5sum for a message object of type '<getNextPose-request>"
  "380505756efd551cb201fc9bfa85fa88")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getNextPose-request)))
  "Returns md5sum for a message object of type 'getNextPose-request"
  "380505756efd551cb201fc9bfa85fa88")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getNextPose-request>)))
  "Returns full string definition for message of type '<getNextPose-request>"
  (cl:format cl:nil "geometry_msgs/Pose currentPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getNextPose-request)))
  "Returns full string definition for message of type 'getNextPose-request"
  (cl:format cl:nil "geometry_msgs/Pose currentPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getNextPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'currentPose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getNextPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getNextPose-request
    (cl:cons ':currentPose (currentPose msg))
))
;//! \htmlinclude getNextPose-response.msg.html

(cl:defclass <getNextPose-response> (roslisp-msg-protocol:ros-message)
  ((commandPose
    :reader commandPose
    :initarg :commandPose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (learningFinished
    :reader learningFinished
    :initarg :learningFinished
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass getNextPose-response (<getNextPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getNextPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getNextPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_srvs-srv:<getNextPose-response> is deprecated: use re_srvs-srv:getNextPose-response instead.")))

(cl:ensure-generic-function 'commandPose-val :lambda-list '(m))
(cl:defmethod commandPose-val ((m <getNextPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:commandPose-val is deprecated.  Use re_srvs-srv:commandPose instead.")
  (commandPose m))

(cl:ensure-generic-function 'learningFinished-val :lambda-list '(m))
(cl:defmethod learningFinished-val ((m <getNextPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_srvs-srv:learningFinished-val is deprecated.  Use re_srvs-srv:learningFinished instead.")
  (learningFinished m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getNextPose-response>) ostream)
  "Serializes a message object of type '<getNextPose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'commandPose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'learningFinished) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getNextPose-response>) istream)
  "Deserializes a message object of type '<getNextPose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'commandPose) istream)
    (cl:setf (cl:slot-value msg 'learningFinished) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getNextPose-response>)))
  "Returns string type for a service object of type '<getNextPose-response>"
  "re_srvs/getNextPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getNextPose-response)))
  "Returns string type for a service object of type 'getNextPose-response"
  "re_srvs/getNextPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getNextPose-response>)))
  "Returns md5sum for a message object of type '<getNextPose-response>"
  "380505756efd551cb201fc9bfa85fa88")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getNextPose-response)))
  "Returns md5sum for a message object of type 'getNextPose-response"
  "380505756efd551cb201fc9bfa85fa88")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getNextPose-response>)))
  "Returns full string definition for message of type '<getNextPose-response>"
  (cl:format cl:nil "geometry_msgs/Pose commandPose~%bool learningFinished~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getNextPose-response)))
  "Returns full string definition for message of type 'getNextPose-response"
  (cl:format cl:nil "geometry_msgs/Pose commandPose~%bool learningFinished~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getNextPose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'commandPose))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getNextPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getNextPose-response
    (cl:cons ':commandPose (commandPose msg))
    (cl:cons ':learningFinished (learningFinished msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getNextPose)))
  'getNextPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getNextPose)))
  'getNextPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getNextPose)))
  "Returns string type for a service object of type '<getNextPose>"
  "re_srvs/getNextPose")