package ecmc;

public interface sudo_driver_input_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ecmc/sudo_driver_input_msg";
  static final java.lang.String _DEFINITION = "Header header\n\nfloat64 target_accel\nfloat64 wheel_angle\nbool aeb_override\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getTargetAccel();
  void setTargetAccel(double value);
  double getWheelAngle();
  void setWheelAngle(double value);
  boolean getAebOverride();
  void setAebOverride(boolean value);
}
