; Auto-generated. Do not edit!


(cl:in-package re_vision-srv)


;//! \htmlinclude SearchFor-request.msg.html

(cl:defclass <SearchFor-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (Image
    :reader Image
    :initarg :Image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (Objects
    :reader Objects
    :initarg :Objects
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (MaxPointsPerObject
    :reader MaxPointsPerObject
    :initarg :MaxPointsPerObject
    :type cl:integer
    :initform 0))
)

(cl:defclass SearchFor-request (<SearchFor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SearchFor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SearchFor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_vision-srv:<SearchFor-request> is deprecated: use re_vision-srv:SearchFor-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SearchFor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_vision-srv:header-val is deprecated.  Use re_vision-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'Image-val :lambda-list '(m))
(cl:defmethod Image-val ((m <SearchFor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_vision-srv:Image-val is deprecated.  Use re_vision-srv:Image instead.")
  (Image m))

(cl:ensure-generic-function 'Objects-val :lambda-list '(m))
(cl:defmethod Objects-val ((m <SearchFor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_vision-srv:Objects-val is deprecated.  Use re_vision-srv:Objects instead.")
  (Objects m))

(cl:ensure-generic-function 'MaxPointsPerObject-val :lambda-list '(m))
(cl:defmethod MaxPointsPerObject-val ((m <SearchFor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_vision-srv:MaxPointsPerObject-val is deprecated.  Use re_vision-srv:MaxPointsPerObject instead.")
  (MaxPointsPerObject m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SearchFor-request>) ostream)
  "Serializes a message object of type '<SearchFor-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Image) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Objects))))
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
   (cl:slot-value msg 'Objects))
  (cl:let* ((signed (cl:slot-value msg 'MaxPointsPerObject)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SearchFor-request>) istream)
  "Deserializes a message object of type '<SearchFor-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Image) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'MaxPointsPerObject) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SearchFor-request>)))
  "Returns string type for a service object of type '<SearchFor-request>"
  "re_vision/SearchForRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchFor-request)))
  "Returns string type for a service object of type 'SearchFor-request"
  "re_vision/SearchForRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SearchFor-request>)))
  "Returns md5sum for a message object of type '<SearchFor-request>"
  "1750b3eb6ab327ff73f77a82f3fd6f57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SearchFor-request)))
  "Returns md5sum for a message object of type 'SearchFor-request"
  "1750b3eb6ab327ff73f77a82f3fd6f57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SearchFor-request>)))
  "Returns full string definition for message of type '<SearchFor-request>"
  (cl:format cl:nil "Header header~%sensor_msgs/Image Image~%string[] Objects~%int32 MaxPointsPerObject~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SearchFor-request)))
  "Returns full string definition for message of type 'SearchFor-request"
  (cl:format cl:nil "Header header~%sensor_msgs/Image Image~%string[] Objects~%int32 MaxPointsPerObject~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SearchFor-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Image))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SearchFor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SearchFor-request
    (cl:cons ':header (header msg))
    (cl:cons ':Image (Image msg))
    (cl:cons ':Objects (Objects msg))
    (cl:cons ':MaxPointsPerObject (MaxPointsPerObject msg))
))
;//! \htmlinclude SearchFor-response.msg.html

(cl:defclass <SearchFor-response> (roslisp-msg-protocol:ros-message)
  ((Detections
    :reader Detections
    :initarg :Detections
    :type (cl:vector re_msgs-msg:DetectedObject)
   :initform (cl:make-array 0 :element-type 're_msgs-msg:DetectedObject :initial-element (cl:make-instance 're_msgs-msg:DetectedObject))))
)

(cl:defclass SearchFor-response (<SearchFor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SearchFor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SearchFor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_vision-srv:<SearchFor-response> is deprecated: use re_vision-srv:SearchFor-response instead.")))

(cl:ensure-generic-function 'Detections-val :lambda-list '(m))
(cl:defmethod Detections-val ((m <SearchFor-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_vision-srv:Detections-val is deprecated.  Use re_vision-srv:Detections instead.")
  (Detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SearchFor-response>) ostream)
  "Serializes a message object of type '<SearchFor-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'Detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SearchFor-response>) istream)
  "Deserializes a message object of type '<SearchFor-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 're_msgs-msg:DetectedObject))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SearchFor-response>)))
  "Returns string type for a service object of type '<SearchFor-response>"
  "re_vision/SearchForResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchFor-response)))
  "Returns string type for a service object of type 'SearchFor-response"
  "re_vision/SearchForResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SearchFor-response>)))
  "Returns md5sum for a message object of type '<SearchFor-response>"
  "1750b3eb6ab327ff73f77a82f3fd6f57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SearchFor-response)))
  "Returns md5sum for a message object of type 'SearchFor-response"
  "1750b3eb6ab327ff73f77a82f3fd6f57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SearchFor-response>)))
  "Returns full string definition for message of type '<SearchFor-response>"
  (cl:format cl:nil "re_msgs/DetectedObject[] Detections~%~%~%~%================================================================================~%MSG: re_msgs/DetectedObject~%# Information of object detected in an image by re_vision~%#~%~%# detected points in the image~%Pixel[] points2d~%# detected 3d points in the camera reference~%geometry_msgs/Point[] points3d~%# pose of the object in the camera reference~%geometry_msgs/Pose pose~%# detected 3d points in the model reference~%geometry_msgs/Point[] points3d_model~%# detected 3d points octave~%int32[] octave~%~%================================================================================~%MSG: re_msgs/Pixel~%# top-left corner: (0,0)~%~%int32 x~%int32 y~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SearchFor-response)))
  "Returns full string definition for message of type 'SearchFor-response"
  (cl:format cl:nil "re_msgs/DetectedObject[] Detections~%~%~%~%================================================================================~%MSG: re_msgs/DetectedObject~%# Information of object detected in an image by re_vision~%#~%~%# detected points in the image~%Pixel[] points2d~%# detected 3d points in the camera reference~%geometry_msgs/Point[] points3d~%# pose of the object in the camera reference~%geometry_msgs/Pose pose~%# detected 3d points in the model reference~%geometry_msgs/Point[] points3d_model~%# detected 3d points octave~%int32[] octave~%~%================================================================================~%MSG: re_msgs/Pixel~%# top-left corner: (0,0)~%~%int32 x~%int32 y~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SearchFor-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SearchFor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SearchFor-response
    (cl:cons ':Detections (Detections msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SearchFor)))
  'SearchFor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SearchFor)))
  'SearchFor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchFor)))
  "Returns string type for a service object of type '<SearchFor>"
  "re_vision/SearchFor")