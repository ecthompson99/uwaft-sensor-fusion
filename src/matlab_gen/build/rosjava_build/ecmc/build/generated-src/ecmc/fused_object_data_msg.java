package ecmc;

public interface fused_object_data_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ecmc/fused_object_data_msg";
  static final java.lang.String _DEFINITION = "Header header\n\nfloat64 accel_x\nfloat64 vel_x\nfloat64 pos_x\nfloat64 pos_y\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getAccelX();
  void setAccelX(double value);
  double getVelX();
  void setVelX(double value);
  double getPosX();
  void setPosX(double value);
  double getPosY();
  void setPosY(double value);
}
