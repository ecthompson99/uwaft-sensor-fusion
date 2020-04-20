package sensor_fusion_testing;

public interface mobileye_object_data_from_matlab extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_fusion_testing/mobileye_object_data_from_matlab";
  static final java.lang.String _DEFINITION = "float64 MeDx\r\nfloat64 MeDy\r\nfloat64 MeVx\r\nfloat64 MeTimestamp\r\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getMeDx();
  void setMeDx(double value);
  double getMeDy();
  void setMeDy(double value);
  double getMeVx();
  void setMeVx(double value);
  double getMeTimestamp();
  void setMeTimestamp(double value);
}
