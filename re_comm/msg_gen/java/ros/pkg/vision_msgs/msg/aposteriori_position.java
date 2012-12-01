/* Auto-generated by genmsg_java.py for file /opt/ros/fuerte/stacks/ias_common/vision_msgs/msg/aposteriori_position.msg */

package ros.pkg.vision_msgs.msg;

import java.nio.ByteBuffer;

public class aposteriori_position extends ros.communication.Message {

  public long objectId;
  public double probability;
  public long position;
  public java.util.ArrayList<ros.pkg.vision_msgs.msg.cop_descriptor> models = new java.util.ArrayList<ros.pkg.vision_msgs.msg.cop_descriptor>();

  public aposteriori_position() {
  }

  public static java.lang.String __s_getDataType() { return "vision_msgs/aposteriori_position"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "37ac3556838265f37bdeb19748c025fb"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "#objects a posteriori position, U. Klank klank@in.tum.de\n" +
"uint64 objectId                  #id of an cop object\n" +
"float64 probability              #approximated a posteriori probability of the object beeing at the position\n" +
"uint64 position                  #lo id of an position\n" +
"cop_descriptor[] models           #list of all models assigned to the returned object\n" +
"================================================================================\n" +
"MSG: vision_msgs/cop_descriptor\n" +
"#Descriptors of models used in cop, U. Klank klank@in.tum.de\n" +
"uint64  object_id          # unique id that could be used for a query\n" +
"string  sem_class          # connected semantic concept\n" +
"string  type               # Class name that was used to generate the corresponding cop descriptor plugin, \n" +
"                           # example are: ShapeModel, ColorClass, DeformShapeModel\n" +
"float64 quality            # the current quality assinged to this descriptor\n" +
"\n" +
"\n" +
"\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public aposteriori_position clone() {
    aposteriori_position c = new aposteriori_position();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 8; // objectId
    __l += 8; // probability
    __l += 8; // position
    __l += 4;
    for(ros.pkg.vision_msgs.msg.cop_descriptor val : models) {
      __l += val.serializationLength();
    }
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    bb.putLong(objectId);
    bb.putDouble(probability);
    bb.putLong(position);
    bb.putInt(models.size());
    for(ros.pkg.vision_msgs.msg.cop_descriptor val : models) {
      val.serialize(bb, seq);
    }
  }

  public void deserialize(ByteBuffer bb) {
    objectId = bb.getLong();
    probability = bb.getDouble();
    position = bb.getLong();

    int __models_len = bb.getInt();
    models = new java.util.ArrayList<ros.pkg.vision_msgs.msg.cop_descriptor>(__models_len);
    for(int __i=0; __i<__models_len; __i++) {
      ros.pkg.vision_msgs.msg.cop_descriptor __tmp = new ros.pkg.vision_msgs.msg.cop_descriptor();
      __tmp.deserialize(bb);
      models.add(__tmp);;
    }
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof aposteriori_position))
      return false;
    aposteriori_position other = (aposteriori_position) o;
    return
      objectId == other.objectId &&
      probability == other.probability &&
      position == other.position &&
      models.equals(other.models) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (int)(this.objectId ^ (this.objectId >>> 32));
    result = prime * result + (int)((tmp = Double.doubleToLongBits(this.probability)) ^ (tmp >>> 32));
    result = prime * result + (int)(this.position ^ (this.position >>> 32));
    result = prime * result + (this.models == null ? 0 : this.models.hashCode());
    return result;
  }
} // class aposteriori_position
