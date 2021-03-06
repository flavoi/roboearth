/* Auto-generated by genmsg_cpp for file /home/flavio/Works/Roboearth_iaslab/stacks/roboearth/re_srvs/srv/RoboEarthExportCopModel.srv */

package ros.pkg.re_srvs.srv;

import java.nio.ByteBuffer;


public class RoboEarthExportCopModel extends ros.communication.Service<RoboEarthExportCopModel.Request, RoboEarthExportCopModel.Response> {

  public static java.lang.String __s_getDataType() { return "re_srvs/RoboEarthExportCopModel"; }
  public static java.lang.String __s_getMD5Sum() { return "6d8d55e47adcbc6e8b3347eeb7ef0727"; }

  public java.lang.String getDataType() { return RoboEarthExportCopModel.__s_getDataType(); }
  public java.lang.String getMD5Sum() { return RoboEarthExportCopModel.__s_getMD5Sum(); }

  public RoboEarthExportCopModel.Request createRequest() {
    return new RoboEarthExportCopModel.Request();
  }

  public RoboEarthExportCopModel.Response createResponse() {
    return new RoboEarthExportCopModel.Response();
  }

static public class Request extends ros.communication.Message {

  public long object_id;

  public Request() {
  }

  public static java.lang.String __s_getDataType() { return "re_srvs/RoboEarthExportCopModelRequest"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "039cdc2e4e021929b349f87d86d0ea70"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getServerMD5Sum() { return "6d8d55e47adcbc6e8b3347eeb7ef0727"; }
  public java.lang.String getServerMD5Sum() { return __s_getServerMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "\n" +
"\n" +
"uint64 object_id\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public Request clone() {
    Request c = new Request();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 8; // object_id
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    bb.putLong(object_id);
  }

  public void deserialize(ByteBuffer bb) {
    object_id = bb.getLong();
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof Request))
      return false;
    Request other = (Request) o;
    return
      object_id == other.object_id &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (int)(this.object_id ^ (this.object_id >>> 32));
    return result;
  }
} // class Request

static public class Response extends ros.communication.Message {

  public short success;

  public Response() {
  }

  public static java.lang.String __s_getDataType() { return "re_srvs/RoboEarthExportCopModelResponse"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "1c39f3eb3ccdcb41c87e9592b334f2a2"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getServerMD5Sum() { return "6d8d55e47adcbc6e8b3347eeb7ef0727"; }
  public java.lang.String getServerMD5Sum() { return __s_getServerMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "byte success\n" +
"\n" +
"\n" +
""; }
  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  public Response clone() {
    Response c = new Response();
    c.deserialize(serialize(0));
    return c;
  }

  public void setTo(ros.communication.Message m) {
    deserialize(m.serialize(0));
  }

  public int serializationLength() {
    int __l = 0;
    __l += 1; // success
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    bb.put((byte)success);
  }

  public void deserialize(ByteBuffer bb) {
    success = (short)(bb.get() & 0xff);
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof Response))
      return false;
    Response other = (Response) o;
    return
      success == other.success &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + this.success;
    return result;
  }
} // class Response

} //class

