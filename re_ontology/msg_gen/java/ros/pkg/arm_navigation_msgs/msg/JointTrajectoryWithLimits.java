/* Auto-generated by genmsg_java.py for file /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg/JointTrajectoryWithLimits.msg */

package ros.pkg.arm_navigation_msgs.msg;

import java.nio.ByteBuffer;

public class JointTrajectoryWithLimits extends ros.communication.Message {

  public ros.pkg.trajectory_msgs.msg.JointTrajectory trajectory = new ros.pkg.trajectory_msgs.msg.JointTrajectory();
  public java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.JointLimits> limits = new java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.JointLimits>();

  public JointTrajectoryWithLimits() {
  }

  public static java.lang.String __s_getDataType() { return "arm_navigation_msgs/JointTrajectoryWithLimits"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "e31e1ba1b3409bbb645c8dfcca5935cd"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "# A trajectory message that encodes joint limits within it.\n" +
"trajectory_msgs/JointTrajectory trajectory\n" +
"\n" +
"# A vector of JointLimit messages.\n" +
"# Each message contains the limits for a specific joint\n" +
"arm_navigation_msgs/JointLimits[] limits\n" +
"\n" +
"================================================================================\n" +
"MSG: trajectory_msgs/JointTrajectory\n" +
"Header header\n" +
"string[] joint_names\n" +
"JointTrajectoryPoint[] points\n" +
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
"MSG: trajectory_msgs/JointTrajectoryPoint\n" +
"float64[] positions\n" +
"float64[] velocities\n" +
"float64[] accelerations\n" +
"duration time_from_start\n" +
"================================================================================\n" +
"MSG: arm_navigation_msgs/JointLimits\n" +
"# This message contains information about limits of a particular joint (or control dimension)\n" +
"string joint_name\n" +
"\n" +
"# true if the joint has position limits\n" +
"bool has_position_limits\n" +
"\n" +
"# min and max position limits\n" +
"float64 min_position\n" +
"float64 max_position\n" +
"\n" +
"# true if joint has velocity limits\n" +
"bool has_velocity_limits\n" +
"\n" +
"# max velocity limit\n" +
"float64 max_velocity\n" +
"# min_velocity is assumed to be -max_velocity\n" +
"\n" +
"# true if joint has acceleration limits\n" +
"bool has_acceleration_limits\n" +
"# max acceleration limit\n" +
"float64 max_acceleration\n" +
"# min_acceleration is assumed to be -max_acceleration\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public JointTrajectoryWithLimits clone() {
    JointTrajectoryWithLimits c = new JointTrajectoryWithLimits();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += trajectory.serializationLength();
    __l += 4;
    for(ros.pkg.arm_navigation_msgs.msg.JointLimits val : limits) {
      __l += val.serializationLength();
    }
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    trajectory.serialize(bb, seq);
    bb.putInt(limits.size());
    for(ros.pkg.arm_navigation_msgs.msg.JointLimits val : limits) {
      val.serialize(bb, seq);
    }
  }

  public void deserialize(ByteBuffer bb) {
    trajectory.deserialize(bb);

    int __limits_len = bb.getInt();
    limits = new java.util.ArrayList<ros.pkg.arm_navigation_msgs.msg.JointLimits>(__limits_len);
    for(int __i=0; __i<__limits_len; __i++) {
      ros.pkg.arm_navigation_msgs.msg.JointLimits __tmp = new ros.pkg.arm_navigation_msgs.msg.JointLimits();
      __tmp.deserialize(bb);
      limits.add(__tmp);;
    }
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof JointTrajectoryWithLimits))
      return false;
    JointTrajectoryWithLimits other = (JointTrajectoryWithLimits) o;
    return
      trajectory.equals(other.trajectory) &&
      limits.equals(other.limits) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.trajectory == null ? 0 : this.trajectory.hashCode());
    result = prime * result + (this.limits == null ? 0 : this.limits.hashCode());
    return result;
  }
} // class JointTrajectoryWithLimits

