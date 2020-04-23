package sensor_fusion;

public interface radar_object_data extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_fusion/radar_object_data";
  static final java.lang.String _DEFINITION = "float64 RadarDx\r\nfloat64 RadarDy\r\nfloat64 RadarVx\r\nfloat64 RadarVy\r\nfloat64 RadarAx\r\nfloat64 RadarTimestamp\r\nfloat64 RadarDxSigma\r\nfloat64 RadarDySigma\r\nfloat64 RadarVxSigma\r\nfloat64 RadarAxSigma\r\n";
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
