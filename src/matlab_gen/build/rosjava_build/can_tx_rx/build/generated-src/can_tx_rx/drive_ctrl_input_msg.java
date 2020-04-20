package can_tx_rx;

public interface drive_ctrl_input_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "can_tx_rx/drive_ctrl_input_msg";
  static final java.lang.String _DEFINITION = "Header header\n\nbool acc_enable\nbool aeb_enable\nbool lc_enable\n\nfloat64 acc_speed_set_point\nfloat64 acc_dist_set_point\n";
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
  double getAccSpeedSetPoint();
  void setAccSpeedSetPoint(double value);
  double getAccDistSetPoint();
  void setAccDistSetPoint(double value);
}
