/* Auto-generated by genmsg_java.py for file /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg/Constraints.msg */

package ros.pkg.arm_navigation_msgs.msg;

import java.nio.ByteBuffer;

public class Constraints extends ros.communication.Message {

  public java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.JointConstraint> joint_constraints = new java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.JointConstraint>();
  public java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.PositionConstraint> position_constraints = new java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.PositionConstraint>();
  public java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.OrientationConstraint> orientation_constraints = new java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.OrientationConstraint>();
  public java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.VisibilityConstraint> visibility_constraints = new java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.VisibilityConstraint>();

  public Constraints() {
  }

  public static java.lang.String __s_getDataType() { return "arm_navigation_msgs/Constraints"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "fe6b6f09c687fd46c05a2de4ca18378a"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "# This message contains a list of motion planning constraints.\n" +
"\n" +
"arm_navigation_msgs/JointConstraint[] joint_constraints\n" +
"arm_navigation_msgs/PositionConstraint[] position_constraints\n" +
"arm_navigation_msgs/OrientationConstraint[] orientation_constraints\n" +
"arm_navigation_msgs/VisibilityConstraint[] visibility_constraints\n" +
"\n" +
"================================================================================\n" +
"MSG: arm_navigation_msgs/JointConstraint\n" +
"# Constrain the position of a joint to be within a certain bound\n" +
"string joint_name\n" +
"\n" +
"# the bound to be achieved is [position - tolerance_below, position + tolerance_above]\n" +
"float64 position\n" +
"float64 tolerance_above\n" +
"float64 tolerance_below\n" +
"\n" +
"# A weighting factor for this constraint\n" +
"float64 weight\n" +
"================================================================================\n" +
"MSG: arm_navigation_msgs/PositionConstraint\n" +
"# This message contains the definition of a position constraint.\n" +
"Header header\n" +
"\n" +
"# The robot link this constraint refers to\n" +
"string link_name\n" +
"\n" +
"# The offset (in the link frame) for the target point on the link we are planning for\n" +
"geometry_msgs/Point target_point_offset\n" +
"\n" +
"# The nominal/target position for the point we are planning for\n" +
"geometry_msgs/Point position\n" +
"\n" +
"# The shape of the bounded region that constrains the position of the end-effector\n" +
"# This region is always centered at the position defined above\n" +
"arm_navigation_msgs/Shape constraint_region_shape\n" +
"\n" +
"# The orientation of the bounded region that constrains the position of the end-effector. \n" +
"# This allows the specification of non-axis aligned constraints\n" +
"geometry_msgs/Quaternion constraint_region_orientation\n" +
"\n" +
"# Constraint weighting factor - a weight for this constraint\n" +
"float64 weight\n" +
"\n" +
"================================================================================\n" +
"MSG: std_msgs/Header\n" +
"# Standard metadata for higher-level stamped data types.\n" +
"# This is generally used to communicate timestamped data \n" +
"# in a particular coordinate frame.\n" +
"# \n" +
"# sequence ID: consecutively increasing ID \n" +
"uint32 seq\n" +
"#Two-integer timestamp that is expressed as:\n" +
"# * stamp.secs: seconds (stamp_secs) since epoch\n" +
"# * stamp.nsecs: nanoseconds since stamp_secs\n" +
"# time-handling sugar is provided by the client library\n" +
"time stamp\n" +
"#Frame this data is associated with\n" +
"# 0: no frame\n" +
"# 1: global frame\n" +
"string frame_id\n" +
"\n" +
"================================================================================\n" +
"MSG: geometry_msgs/Point\n" +
"# This contains the position of a point in free space\n" +
"float64 x\n" +
"float64 y\n" +
"float64 z\n" +
"\n" +
"================================================================================\n" +
"MSG: arm_navigation_msgs/Shape\n" +
"byte SPHERE=0\n" +
"byte BOX=1\n" +
"byte CYLINDER=2\n" +
"byte MESH=3\n" +
"\n" +
"byte type\n" +
"\n" +
"\n" +
"#### define sphere, box, cylinder ####\n" +
"# the origin of each shape is considered at the shape's center\n" +
"\n" +
"# for sphere\n" +
"# radius := dimensions[0]\n" +
"\n" +
"# for cylinder\n" +
"# radius := dimensions[0]\n" +
"# length := dimensions[1]\n" +
"# the length is along the Z axis\n" +
"\n" +
"# for box\n" +
"# size_x := dimensions[0]\n" +
"# size_y := dimensions[1]\n" +
"# size_z := dimensions[2]\n" +
"float64[] dimensions\n" +
"\n" +
"\n" +
"#### define mesh ####\n" +
"\n" +
"# list of triangles; triangle k is defined by tre vertices located\n" +
"# at indices triangles[3k], triangles[3k+1], triangles[3k+2]\n" +
"int32[] triangles\n" +
"geometry_msgs/Point[] vertices\n" +
"\n" +
"================================================================================\n" +
"MSG: geometry_msgs/Quaternion\n" +
"# This represents an orientation in free space in quaternion form.\n" +
"\n" +
"float64 x\n" +
"float64 y\n" +
"float64 z\n" +
"float64 w\n" +
"\n" +
"================================================================================\n" +
"MSG: arm_navigation_msgs/OrientationConstraint\n" +
"# This message contains the definition of an orientation constraint.\n" +
"Header header\n" +
"\n" +
"# The robot link this constraint refers to\n" +
"string link_name\n" +
"\n" +
"# The type of the constraint\n" +
"int32 type\n" +
"int32 LINK_FRAME=0\n" +
"int32 HEADER_FRAME=1\n" +
"\n" +
"# The desired orientation of the robot link specified as a quaternion\n" +
"geometry_msgs/Quaternion orientation\n" +
"\n" +
"# optional RPY error tolerances specified if \n" +
"float64 absolute_roll_tolerance\n" +
"float64 absolute_pitch_tolerance\n" +
"float64 absolute_yaw_tolerance\n" +
"\n" +
"# Constraint weighting factor - a weight for this constraint\n" +
"float64 weight\n" +
"\n" +
"================================================================================\n" +
"MSG: arm_navigation_msgs/VisibilityConstraint\n" +
"# This message contains the definition of a visibility constraint.\n" +
"Header header\n" +
"\n" +
"# The point stamped target that needs to be kept within view of the sensor\n" +
"geometry_msgs/PointStamped target\n" +
"\n" +
"# The local pose of the frame in which visibility is to be maintained\n" +
"# The frame id should represent the robot link to which the sensor is attached\n" +
"# The visual axis of the sensor is assumed to be along the X axis of this frame\n" +
"geometry_msgs/PoseStamped sensor_pose\n" +
"\n" +
"# The deviation (in radians) that will be tolerated\n" +
"# Constraint error will be measured as the solid angle between the \n" +
"# X axis of the frame defined above and the vector between the origin \n" +
"# of the frame defined above and the target location\n" +
"float64 absolute_tolerance\n" +
"\n" +
"\n" +
"================================================================================\n" +
"MSG: geometry_msgs/PointStamped\n" +
"# This represents a Point with reference coordinate frame and timestamp\n" +
"Header header\n" +
"Point point\n" +
"\n" +
"================================================================================\n" +
"MSG: geometry_msgs/PoseStamped\n" +
"# A Pose with reference coordinate frame and timestamp\n" +
"Header header\n" +
"Pose pose\n" +
"\n" +
"================================================================================\n" +
"MSG: geometry_msgs/Pose\n" +
"# A representation of pose in free space, composed of postion and orientation. \n" +
"Point position\n" +
"Quaternion orientation\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public Constraints clone() {
    Constraints c = new Constraints();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 4;
    for(ros.pkg.arm_navigation_msgs.msg.JointConstraint val : joint_constraints) {
      __l += val.serializationLength();
    }
    __l += 4;
    for(ros.pkg.arm_navigation_msgs.msg.PositionConstraint val : position_constraints) {
      __l += val.serializationLength();
    }
    __l += 4;
    for(ros.pkg.arm_navigation_msgs.msg.OrientationConstraint val : orientation_constraints) {
      __l += val.serializationLength();
    }
    __l += 4;
    for(ros.pkg.arm_navigation_msgs.msg.VisibilityConstraint val : visibility_constraints) {
      __l += val.serializationLength();
    }
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    bb.putInt(joint_constraints.size());
    for(ros.pkg.arm_navigation_msgs.msg.JointConstraint val : joint_constraints) {
      val.serialize(bb, seq);
    }
    bb.putInt(position_constraints.size());
    for(ros.pkg.arm_navigation_msgs.msg.PositionConstraint val : position_constraints) {
      val.serialize(bb, seq);
    }
    bb.putInt(orientation_constraints.size());
    for(ros.pkg.arm_navigation_msgs.msg.OrientationConstraint val : orientation_constraints) {
      val.serialize(bb, seq);
    }
    bb.putInt(visibility_constraints.size());
    for(ros.pkg.arm_navigation_msgs.msg.VisibilityConstraint val : visibility_constraints) {
      val.serialize(bb, seq);
    }
  }

  public void deserialize(ByteBuffer bb) {

    int __joint_constraints_len = bb.getInt();
    joint_constraints = new java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.JointConstraint>(__joint_constraints_len);
    for(int __i=0; __i<__joint_constraints_len; __i++) {
      ros.pkg.arm_navigation_msgs.msg.JointConstraint __tmp = new ros.pkg.arm_navigation_msgs.msg.JointConstraint();
      __tmp.deserialize(bb);
      joint_constraints.add(__tmp);;
    }

    int __position_constraints_len = bb.getInt();
    position_constraints = new java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.PositionConstraint>(__position_constraints_len);
    for(int __i=0; __i<__position_constraints_len; __i++) {
      ros.pkg.arm_navigation_msgs.msg.PositionConstraint __tmp = new ros.pkg.arm_navigation_msgs.msg.PositionConstraint();
      __tmp.deserialize(bb);
      position_constraints.add(__tmp);;
    }

    int __orientation_constraints_len = bb.getInt();
    orientation_constraints = new java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.OrientationConstraint>(__orientation_constraints_len);
    for(int __i=0; __i<__orientation_constraints_len; __i++) {
      ros.pkg.arm_navigation_msgs.msg.OrientationConstraint __tmp = new ros.pkg.arm_navigation_msgs.msg.OrientationConstraint();
      __tmp.deserialize(bb);
      orientation_constraints.add(__tmp);;
    }

    int __visibility_constraints_len = bb.getInt();
    visibility_constraints = new java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.VisibilityConstraint>(__visibility_constraints_len);
    for(int __i=0; __i<__visibility_constraints_len; __i++) {
      ros.pkg.arm_navigation_msgs.msg.VisibilityConstraint __tmp = new ros.pkg.arm_navigation_msgs.msg.VisibilityConstraint();
      __tmp.deserialize(bb);
      visibility_constraints.add(__tmp);;
    }
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof Constraints))
      return false;
    Constraints other = (Constraints) o;
    return
      joint_constraints.equals(other.joint_constraints) &&
      position_constraints.equals(other.position_constraints) &&
      orientation_constraints.equals(other.orientation_constraints) &&
      visibility_constraints.equals(other.visibility_constraints) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.joint_constraints == null ? 0 : this.joint_constraints.hashCode());
    result = prime * result + (this.position_constraints == null ? 0 : this.position_constraints.hashCode());
    result = prime * result + (this.orientation_constraints == null ? 0 : this.orientation_constraints.hashCode());
    result = prime * result + (this.visibility_constraints == null ? 0 : this.visibility_constraints.hashCode());
    return result;
  }
} // class Constraints
