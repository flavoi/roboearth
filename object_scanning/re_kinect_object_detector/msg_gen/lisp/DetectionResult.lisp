; Auto-generated. Do not edit!


(cl:in-package re_kinect_object_detector-msg)


;//! \htmlinclude DetectionResult.msg.html

(cl:defclass <DetectionResult> (roslisp-msg-protocol:ros-message)
  ((Image
    :reader Image
    :initarg :Image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (ObjectNames
    :reader ObjectNames
    :initarg :ObjectNames
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (Detections
    :reader Detections
    :initarg :Detections
    :type (cl:vector re_msgs-msg:DetectedObject)
   :initform (cl:make-array 0 :element-type 're_msgs-msg:DetectedObject :initial-element (cl:make-instance 're_msgs-msg:DetectedObject))))
)

(cl:defclass DetectionResult (<DetectionResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectionResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectionResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name re_kinect_object_detector-msg:<DetectionResult> is deprecated: use re_kinect_object_detector-msg:DetectionResult instead.")))

(cl:ensure-generic-function 'Image-val :lambda-list '(m))
(cl:defmethod Image-val ((m <DetectionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_kinect_object_detector-msg:Image-val is deprecated.  Use re_kinect_object_detector-msg:Image instead.")
  (Image m))

(cl:ensure-generic-function 'ObjectNames-val :lambda-list '(m))
(cl:defmethod ObjectNames-val ((m <DetectionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_kinect_object_detector-msg:ObjectNames-val is deprecated.  Use re_kinect_object_detector-msg:ObjectNames instead.")
  (ObjectNames m))

(cl:ensure-generic-function 'Detections-val :lambda-list '(m))
(cl:defmethod Detections-val ((m <DetectionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader re_kinect_object_detector-msg:Detections-val is deprecated.  Use re_kinect_object_detector-msg:Detections instead.")
  (Detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectionResult>) ostream)
  "Serializes a message object of type '<DetectionResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Image) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ObjectNames))))
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
   (cl:slot-value msg 'ObjectNames))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'Detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectionResult>) istream)
  "Deserializes a message object of type '<DetectionResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Image) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ObjectNames) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ObjectNames)))
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
  (cl:setf (cl:slot-value msg 'Detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 're_msgs-msg:DetectedObject))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectionResult>)))
  "Returns string type for a message object of type '<DetectionResult>"
  "re_kinect_object_detector/DetectionResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectionResult)))
  "Returns string type for a message object of type 'DetectionResult"
  "re_kinect_object_detector/DetectionResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectionResult>)))
  "Returns md5sum for a message object of type '<DetectionResult>"
  "ff5eaa0c7838288761622bd26a33514f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectionResult)))
  "Returns md5sum for a message object of type 'DetectionResult"
  "ff5eaa0c7838288761622bd26a33514f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectionResult>)))
  "Returns full string definition for message of type '<DetectionResult>"
  (cl:format cl:nil "sensor_msgs/Image Image~%string[] ObjectNames~%re_msgs/DetectedObject[] Detections~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: re_msgs/DetectedObject~%# Information of object detected in an image by re_vision~%#~%~%# detected points in the image~%Pixel[] points2d~%# detected 3d points in the camera reference~%geometry_msgs/Point[] points3d~%# pose of the object in the camera reference~%geometry_msgs/Pose pose~%# detected 3d points in the model reference~%geometry_msgs/Point[] points3d_model~%# detected 3d points octave~%int32[] octave~%~%================================================================================~%MSG: re_msgs/Pixel~%# top-left corner: (0,0)~%~%int32 x~%int32 y~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectionResult)))
  "Returns full string definition for message of type 'DetectionResult"
  (cl:format cl:nil "sensor_msgs/Image Image~%string[] ObjectNames~%re_msgs/DetectedObject[] Detections~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in src/image_encodings.cpp~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: re_msgs/DetectedObject~%# Information of object detected in an image by re_vision~%#~%~%# detected points in the image~%Pixel[] points2d~%# detected 3d points in the camera reference~%geometry_msgs/Point[] points3d~%# pose of the object in the camera reference~%geometry_msgs/Pose pose~%# detected 3d points in the model reference~%geometry_msgs/Point[] points3d_model~%# detected 3d points octave~%int32[] octave~%~%================================================================================~%MSG: re_msgs/Pixel~%# top-left corner: (0,0)~%~%int32 x~%int32 y~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectionResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Image))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ObjectNames) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectionResult>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectionResult
    (cl:cons ':Image (Image msg))
    (cl:cons ':ObjectNames (ObjectNames msg))
    (cl:cons ':Detections (Detections msg))
))
