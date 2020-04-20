package sensor_fusion;

public interface radar_object_data extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_fusion/radar_object_data";
  static final java.lang.String _DEFINITION = "float64 radar_dx\r\nfloat64 radar_dy\r\nfloat64 radar_vx\r\nfloat64 radar_vy\r\nfloat64 radar_ax\r\nuint8 radar_target_num\r\nfloat64 radar_timestamp\r\nfloat64 radar_dx_sigma\r\nfloat64 radar_dy_sigma\r\nfloat64 radar_vx_sigma\r\nfloat64 radar_ax_sigma\r\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getRadarDx();
  void setRadarDx(double value);
  double getRadarDy();
  void setRadarDy(double value);
  double getRadarVx();
  void setRadarVx(double value);
  double getRadarVy();
  void setRadarVy(double value);
  double getRadarAx();
  void setRadarAx(double value);
  byte getRadarTargetNum();
  void setRadarTargetNum(byte value);
  double getRadarTimestamp();
  void setRadarTimestamp(double value);
  double getRadarDxSigma();
  void setRadarDxSigma(double value);
  double getRadarDySigma();
  void setRadarDySigma(double value);
  double getRadarVxSigma();
  void setRadarVxSigma(double value);
  double getRadarAxSigma();
  void setRadarAxSigma(double value);
}
