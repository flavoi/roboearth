/* Auto-generated by genmsg_java.py for file /opt/ros/fuerte/stacks/arm_navigation/arm_navigation_msgs/msg/CollisionOperation.msg */

package ros.pkg.arm_navigation_msgs.msg;

import java.nio.ByteBuffer;

public class CollisionOperation extends ros.communication.Message {
  static public final java.lang.String COLLISION_SET_ALL = "\"all\"";
  static public final java.lang.String COLLISION_SET_OBJECTS = "\"objects\"";
  static public final java.lang.String COLLISION_SET_ATTACHED_OBJECTS = "\"attached\"";
  static public final int DISABLE = 0;
  static public final int ENABLE = 1;

  public java.lang.String object1 = new java.lang.String();
  public java.lang.String object2 = new java.lang.String();
  public double penetration_distance;
  public int operation;

  public CollisionOperation() {
  }

  public static java.lang.String __s_getDataType() { return "arm_navigation_msgs/CollisionOperation"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "e0cf3073b26bd86266c918a0c779f8a2"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "# A definition of a collision operation\n" +
"# E.g. (\"gripper\",COLLISION_SET_ALL,ENABLE) will enable collisions \n" +
"# between the gripper and all objects in the collision space\n" +
"\n" +
"string object1\n" +
"string object2\n" +
"string COLLISION_SET_ALL=\"all\"\n" +
"string COLLISION_SET_OBJECTS=\"objects\"\n" +
"string COLLISION_SET_ATTACHED_OBJECTS=\"attached\"\n" +
"\n" +
"# The penetration distance to which collisions are allowed. This is 0.0 by default.\n" +
"float64 penetration_distance\n" +
"\n" +
"# Flag that determines whether collisions will be enabled or disabled for the pair of objects specified above\n" +
"int32 operation\n" +
"int32 DISABLE=0\n" +
"int32 ENABLE=1\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public CollisionOperation clone() {
    CollisionOperation c = new CollisionOperation();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 4 + object1.length();
    __l += 4 + object2.length();
    __l += 8; // penetration_distance
    __l += 4; // operation
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    Serialization.writeString(bb, object1);
    Serialization.writeString(bb, object2);
    bb.putDouble(penetration_distance);
    bb.putInt(operation);
  }

  public void deserialize(ByteBuffer bb) {
    object1 = Serialization.readString(bb);
    object2 = Serialization.readString(bb);
    penetration_distance = bb.getDouble();
    operation = bb.getInt();
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof CollisionOperation))
      return false;
    CollisionOperation other = (CollisionOperation) o;
    return
      object1.equals(other.object1) &&
      object2.equals(other.object2) &&
      penetration_distance == other.penetration_distance &&
      operation == other.operation &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.object1 == null ? 0 : this.object1.hashCode());
    result = prime * result + (this.object2 == null ? 0 : this.object2.hashCode());
    result = prime * result + (int)((tmp = Double.doubleToLongBits(this.penetration_distance)) ^ (tmp >>> 32));
    result = prime * result + this.operation;
    return result;
  }
} // class CollisionOperation
