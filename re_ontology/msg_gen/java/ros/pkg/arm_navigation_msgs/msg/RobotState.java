/* Auto-generated by genmsg_java.py for file /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg/RobotState.msg */

package ros.pkg.arm_navigation_msgs.msg;

import java.nio.ByteBuffer;

public class RobotState extends ros.communication.Message {

  public ros.pkg.sensor_msgs.msg.JointState joint_state = new ros.pkg.sensor_msgs.msg.JointState();
  public ros.pkg.arm_navigation_msgs.msg.MultiDOFJointState multi_dof_joint_state = new ros.pkg.arm_navigation_msgs.msg.MultiDOFJointState();

  public RobotState() {
  }

  public static java.lang.String __s_getDataType() { return "arm_navigation_msgs/RobotState"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "970d46b2ca41b9686adbdaeb592d97a7"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "# This message contains information about the robot state, i.e. the positions of its joints and links\n" +
"sensor_msgs/JointState joint_state\n" +
"arm_navigation_msgs/MultiDOFJointState multi_dof_joint_state\n" +
"\n" +
"================================================================================\n" +
"MSG: sensor_msgs/JointState\n" +
"# This is a message that holds data to describe the state of a set of torque controlled joints. \n" +
"#\n" +
"# The state of each joint (revolute or prismatic) is defined by:\n" +
"#  * the position of the joint (rad or m),\n" +
"#  * the velocity of the joint (rad/s or m/s) and \n" +
"#  * the effort that is applied in the joint (Nm or N).\n" +
"#\n" +
"# Each joint is uniquely identified by its name\n" +
"# The header specifies the time at which the joint states were recorded. All the joint states\n" +
"# in one message have to be recorded at the same time.\n" +
"#\n" +
"# This message consists of a multiple arrays, one for each part of the joint state. \n" +
"# The goal is to make each of the fields optional. When e.g. your joints have no\n" +
"# effort associated with them, you can leave the effort array empty. \n" +
"#\n" +
"# All arrays in this message should have the same size, or be empty.\n" +
"# This is the only way to uniquely associate the joint name with the correct\n" +
"# states.\n" +
"\n" +
"\n" +
"Header header\n" +
"\n" +
"string[] name\n" +
"float64[] position\n" +
"float64[] velocity\n" +
"float64[] effort\n" +
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
"MSG: arm_navigation_msgs/MultiDOFJointState\n" +
"#A representation of a multi-dof joint state\n" +
"time stamp\n" +
"string[] joint_names\n" +
"string[] frame_ids\n" +
"string[] child_frame_ids\n" +
"geometry_msgs/Pose[] poses\n" +
"\n" +
"================================================================================\n" +
"MSG: geometry_msgs/Pose\n" +
"# A representation of pose in free space, composed of postion and orientation. \n" +
"Point position\n" +
"Quaternion orientation\n" +
"\n" +
"================================================================================\n" +
"MSG: geometry_msgs/Point\n" +
"# This contains the position of a point in free space\n" +
"float64 x\n" +
"float64 y\n" +
"float64 z\n" +
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
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public RobotState clone() {
    RobotState c = new RobotState();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += joint_state.serializationLength();
    __l += multi_dof_joint_state.serializationLength();
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    joint_state.serialize(bb, seq);
    multi_dof_joint_state.serialize(bb, seq);
  }

  public void deserialize(ByteBuffer bb) {
    joint_state.deserialize(bb);
    multi_dof_joint_state.deserialize(bb);
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof RobotState))
      return false;
    RobotState other = (RobotState) o;
    return
      joint_state.equals(other.joint_state) &&
      multi_dof_joint_state.equals(other.multi_dof_joint_state) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.joint_state == null ? 0 : this.joint_state.hashCode());
    result = prime * result + (this.multi_dof_joint_state == null ? 0 : this.multi_dof_joint_state.hashCode());
    return result;
  }
} // class RobotState

