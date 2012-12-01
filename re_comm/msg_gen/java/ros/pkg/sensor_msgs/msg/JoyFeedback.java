/* Auto-generated by genmsg_java.py for file /opt/ros/fuerte/share/sensor_msgs/msg/JoyFeedback.msg */

package ros.pkg.sensor_msgs.msg;

import java.nio.ByteBuffer;

public class JoyFeedback extends ros.communication.Message {
  static public final short TYPE_LED = 0;
  static public final short TYPE_RUMBLE = 1;
  static public final short TYPE_BUZZER = 2;

  public short type;
  public short id;
  public float intensity;

  public JoyFeedback() {
  }

  public static java.lang.String __s_getDataType() { return "sensor_msgs/JoyFeedback"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "f4dcd73460360d98f36e55ee7f2e46f1"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "# Declare of the type of feedback\n" +
"uint8 TYPE_LED    = 0\n" +
"uint8 TYPE_RUMBLE = 1\n" +
"uint8 TYPE_BUZZER = 2\n" +
"\n" +
"uint8 type\n" +
"\n" +
"# This will hold an id number for each type of each feedback.\n" +
"# Example, the first led would be id=0, the second would be id=1\n" +
"uint8 id\n" +
"\n" +
"# Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is\n" +
"# actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.\n" +
"float32 intensity\n" +
"\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public JoyFeedback clone() {
    JoyFeedback c = new JoyFeedback();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 1; // type
    __l += 1; // id
    __l += 4; // intensity
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    bb.put((byte)type);
    bb.put((byte)id);
    bb.putFloat(intensity);
  }

  public void deserialize(ByteBuffer bb) {
    type = (short)(bb.get() & 0xff);
    id = (short)(bb.get() & 0xff);
    intensity = bb.getFloat();
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof JoyFeedback))
      return false;
    JoyFeedback other = (JoyFeedback) o;
    return
      type == other.type &&
      id == other.id &&
      intensity == other.intensity &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + this.type;
    result = prime * result + this.id;
    result = prime * result + Float.floatToIntBits(this.intensity);
    return result;
  }
} // class JoyFeedback
