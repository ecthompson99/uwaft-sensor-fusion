package ecmc;

public interface can_comms_data_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ecmc/can_comms_data_msg";
  static final java.lang.String _DEFINITION = "Header header\n\nbool acc_switch_status\nbool aeb_switch_status\nbool lc_switch_status\n\nfloat64 target_accel\nbool acc_valid\nbool hold_target_speed\nfloat64 speed_setpoint\n\nbool aeb_valid\nbool aeb_override\n\nbool lc_valid\nfloat64 wheel_angle\n\nuint64 alive_rolling_counter\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getAccSwitchStatus();
  void setAccSwitchStatus(boolean value);
  boolean getAebSwitchStatus();
  void setAebSwitchStatus(boolean value);
  boolean getLcSwitchStatus();
  void setLcSwitchStatus(boolean value);
  double getTargetAccel();
  void setTargetAccel(double value);
  boolean getAccValid();
  void setAccValid(boolean value);
  boolean getHoldTargetSpeed();
  void setHoldTargetSpeed(boolean value);
  double getSpeedSetpoint();
  void setSpeedSetpoint(double value);
  boolean getAebValid();
  void setAebValid(boolean value);
  boolean getAebOverride();
  void setAebOverride(boolean value);
  boolean getLcValid();
  void setLcValid(boolean value);
  double getWheelAngle();
  void setWheelAngle(double value);
  long getAliveRollingCounter();
  void setAliveRollingCounter(long value);
}
