/* Auto-generated by genmsg_java.py for file /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_msgs/msg/StringArray.msg */

package ros.pkg.re_msgs.msg;

import java.nio.ByteBuffer;

public class StringArray extends ros.communication.Message {

  public java.util.ArrayList<java.lang.String> list = new java.util.ArrayList<java.lang.String>();

  public StringArray() {
  }

  public static java.lang.String __s_getDataType() { return "re_msgs/StringArray"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "8e09fa9aad51508cfdcab3b64f048ce0"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "# A StringArray message contains an array of strings. This is used by other \n" +
"# message/service declarations in order to create 2-dimensional string \n" +
"# arrays with different lengths for one dimension (StringArray[])\n" +
"string[] list   # array of strings\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public StringArray clone() {
    StringArray c = new StringArray();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 4;
    for(java.lang.String val : list) {
      __l += 4 + val.length();
    }
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    bb.putInt(list.size());
    for(java.lang.String val : list) {
      Serialization.writeString(bb, val);
    }
  }

  public void deserialize(ByteBuffer bb) {

    int __list_len = bb.getInt();
    list = new java.util.ArrayList<java.lang.String>(__list_len);
    for(int __i=0; __i<__list_len; __i++) {
      list.add(Serialization.readString(bb));
    }
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof StringArray))
      return false;
    StringArray other = (StringArray) o;
    return
      list.equals(other.list) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.list == null ? 0 : this.list.hashCode());
    return result;
  }
} // class StringArray

