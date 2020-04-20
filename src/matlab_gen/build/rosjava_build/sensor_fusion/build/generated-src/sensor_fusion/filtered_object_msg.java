package sensor_fusion;

public interface filtered_object_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_fusion/filtered_object_msg";
  static final java.lang.String _DEFINITION = "Header header\n\nuint8 obj_id\nfloat64 obj_dx\nuint8 obj_lane\nfloat64 obj_vx\nfloat64 obj_dy\nfloat64 obj_ax\nbool obj_path\nfloat64 obj_vy\nfloat64 obj_timestamp\nuint8 obj_count\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getObjId();
  void setObjId(byte value);
  double getObjDx();
  void setObjDx(double value);
  byte getObjLane();
  void setObjLane(byte value);
  double getObjVx();
  void setObjVx(double value);
  double getObjDy();
  void setObjDy(double value);
  double getObjAx();
  void setObjAx(double value);
  boolean getObjPath();
  void setObjPath(boolean value);
  double getObjVy();
  void setObjVy(double value);
  double getObjTimestamp();
  void setObjTimestamp(double value);
  byte getObjCount();
  void setObjCount(byte value);
}
