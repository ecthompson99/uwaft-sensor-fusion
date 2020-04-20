package ecmc;

public interface drive_control_input_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ecmc/drive_control_input_msg";
  static final java.lang.String _DEFINITION = "Header header\n\nbool acc_enable\nbool aeb_enable\nbool lc_enable\n\nfloat64 acc_speed_setpoint\nfloat64 acc_dist_setpoint\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getAccEnable();
  void setAccEnable(boolean value);
  boolean getAebEnable();
  void setAebEnable(boolean value);
  boolean getLcEnable();
  void setLcEnable(boolean value);
  double getAccSpeedSetpoint();
  void setAccSpeedSetpoint(double value);
  double getAccDistSetpoint();
  void setAccDistSetpoint(double value);
}
