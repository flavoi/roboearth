/* Auto-generated by genmsg_java.py for file /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg/AllowedContactSpecification.msg */

package ros.pkg.arm_navigation_msgs.msg;

import java.nio.ByteBuffer;

public class AllowedContactSpecification extends ros.communication.Message {

  public java.lang.String name = new java.lang.String();
  public ros.pkg.arm_navigation_msgs.msg.Shape shape = new ros.pkg.arm_navigation_msgs.msg.Shape();
  public ros.pkg.geometry_msgs.msg.PoseStamped pose_stamped = new ros.pkg.geometry_msgs.msg.PoseStamped();
  public java.util.ArrayList<java.lang.String> link_names = new java.util.ArrayList<java.lang.String>();
  public double penetration_depth;

  public AllowedContactSpecification() {
  }

  public static java.lang.String __s_getDataType() { return "arm_navigation_msgs/AllowedContactSpecification"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "81f9b47ac49a467ae008d3d9485628a3"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "# The names of the regions\n" +
"string name\n" +
"\n" +
"# The shape of the region in the environment\n" +
"arm_navigation_msgs/Shape shape\n" +
"\n" +
"# The pose of the space defining the region\n" +
"geometry_msgs/PoseStamped pose_stamped\n" +
"\n" +
"# The set of links that will be allowed to have penetration contact within this region\n" +
"string[] link_names\n" +
"\n" +
"# The maximum penetration depth allowed for every link\n" +
"float64 penetration_depth\n" +
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
"MSG: geometry_msgs/Point\n" +
"# This contains the position of a point in free space\n" +
"float64 x\n" +
"float64 y\n" +
"float64 z\n" +
"\n" +
"================================================================================\n" +
"MSG: geometry_msgs/PoseStamped\n" +
"# A Pose with reference coordinate frame and timestamp\n" +
"Header header\n" +
"Pose pose\n" +
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
"MSG: geometry_msgs/Pose\n" +
"# A representation of pose in free space, composed of postion and orientation. \n" +
"Point position\n" +
"Quaternion orientation\n" +
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

  public AllowedContactSpecification clone() {
    AllowedContactSpecification c = new AllowedContactSpecification();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 4 + name.length();
    __l += shape.serializationLength();
    __l += pose_stamped.serializationLength();
    __l += 4;
    for(java.lang.String val : link_names) {
      __l += 4 + val.length();
    }
    __l += 8; // penetration_depth
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    Serialization.writeString(bb, name);
    shape.serialize(bb, seq);
    pose_stamped.serialize(bb, seq);
    bb.putInt(link_names.size());
    for(java.lang.String val : link_names) {
      Serialization.writeString(bb, val);
    }
    bb.putDouble(penetration_depth);
  }

  public void deserialize(ByteBuffer bb) {
    name = Serialization.readString(bb);
    shape.deserialize(bb);
    pose_stamped.deserialize(bb);

    int __link_names_len = bb.getInt();
    link_names = new java.util.ArrayList<java.lang.String>(__link_names_len);
    for(int __i=0; __i<__link_names_len; __i++) {
      link_names.add(Serialization.readString(bb));
    }
    penetration_depth = bb.getDouble();
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof AllowedContactSpecification))
      return false;
    AllowedContactSpecification other = (AllowedContactSpecification) o;
    return
      name.equals(other.name) &&
      shape.equals(other.shape) &&
      pose_stamped.equals(other.pose_stamped) &&
      link_names.equals(other.link_names) &&
      penetration_depth == other.penetration_depth &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.name == null ? 0 : this.name.hashCode());
    result = prime * result + (this.shape == null ? 0 : this.shape.hashCode());
    result = prime * result + (this.pose_stamped == null ? 0 : this.pose_stamped.hashCode());
    result = prime * result + (this.link_names == null ? 0 : this.link_names.hashCode());
    result = prime * result + (int)((tmp = Double.doubleToLongBits(this.penetration_depth)) ^ (tmp >>> 32));
    return result;
  }
} // class AllowedContactSpecification
