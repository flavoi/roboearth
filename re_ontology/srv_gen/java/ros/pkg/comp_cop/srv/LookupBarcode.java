/* Auto-generated by genmsg_cpp for file /home/flavio/Works/Roboearth_iaslab/stacks/knowrob/comp_cop/srv/LookupBarcode.srv */

package ros.pkg.comp_cop.srv;

import java.nio.ByteBuffer;


public class LookupBarcode extends ros.communication.Service<LookupBarcode.Request, LookupBarcode.Response> {

  public static java.lang.String __s_getDataType() { return "comp_cop/LookupBarcode"; }
  public static java.lang.String __s_getMD5Sum() { return "591cf499868ba65dc38bba41446b03a9"; }

  public java.lang.String getDataType() { return LookupBarcode.__s_getDataType(); }
  public java.lang.String getMD5Sum() { return LookupBarcode.__s_getMD5Sum(); }

  public LookupBarcode.Request createRequest() {
    return new LookupBarcode.Request();
  }

  public LookupBarcode.Response createResponse() {
    return new LookupBarcode.Response();
  }

static public class Request extends ros.communication.Message {

  public java.lang.String ean = new java.lang.String();

  public Request() {
  }

  public static java.lang.String __s_getDataType() { return "comp_cop/LookupBarcodeRequest"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "8fe79eda5fcb64e5c9526fa12bf35185"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getServerMD5Sum() { return "591cf499868ba65dc38bba41446b03a9"; }
  public java.lang.String getServerMD5Sum() { return __s_getServerMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "string ean\n" +
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
    __l += 4 + ean.length();
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    Serialization.writeString(bb, ean);
  }

  public void deserialize(ByteBuffer bb) {
    ean = Serialization.readString(bb);
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof Request))
      return false;
    Request other = (Request) o;
    return
      ean.equals(other.ean) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.ean == null ? 0 : this.ean.hashCode());
    return result;
  }
} // class Request

static public class Response extends ros.communication.Message {

  public java.lang.String objectclass = new java.lang.String();

  public Response() {
  }

  public static java.lang.String __s_getDataType() { return "comp_cop/LookupBarcodeResponse"; }
  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "bed7b174870245b52201d473100ab36a"; }
  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getServerMD5Sum() { return "591cf499868ba65dc38bba41446b03a9"; }
  public java.lang.String getServerMD5Sum() { return __s_getServerMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "string objectclass\n" +
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
    __l += 4 + objectclass.length();
    return __l;
  }

  public void serialize(ByteBuffer bb, int seq) {
    Serialization.writeString(bb, objectclass);
  }

  public void deserialize(ByteBuffer bb) {
    objectclass = Serialization.readString(bb);
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof Response))
      return false;
    Response other = (Response) o;
    return
      objectclass.equals(other.objectclass) &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + (this.objectclass == null ? 0 : this.objectclass.hashCode());
    return result;
  }
} // class Response

} //class

