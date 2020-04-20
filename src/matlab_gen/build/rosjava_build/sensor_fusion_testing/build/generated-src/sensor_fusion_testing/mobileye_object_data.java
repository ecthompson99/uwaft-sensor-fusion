package sensor_fusion_testing;

public interface mobileye_object_data extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_fusion_testing/mobileye_object_data";
  static final java.lang.String _DEFINITION = "float64 me_dx\r\nfloat64 me_dy\r\nfloat64 me_vx\r\nfloat64 me_ax\r\nuint8 me_object_id\r\nuint8 me_object_lane\r\nfloat64 me_timestamp\r\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getMeDx();
  void setMeDx(double value);
  double getMeDy();
  void setMeDy(double value);
  double getMeVx();
  void setMeVx(double value);
  double getMeAx();
  void setMeAx(double value);
  byte getMeObjectId();
  void setMeObjectId(byte value);
  byte getMeObjectLane();
  void setMeObjectLane(byte value);
  double getMeTimestamp();
  void setMeTimestamp(double value);
}
