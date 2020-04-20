package ecmc;

public interface raw_sensor_object_data_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ecmc/raw_sensor_object_data_msg";
  static final java.lang.String _DEFINITION = "Header header\n\nuint8 radar_num\nuint8 num_objects\nfloat64[] accel_x\nfloat64[] vel_x\nfloat64[] pos_x\nfloat64[] pos_y\nfloat64[] exist_prob\nbool[] valid\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getRadarNum();
  void setRadarNum(byte value);
  byte getNumObjects();
  void setNumObjects(byte value);
  double[] getAccelX();
  void setAccelX(double[] value);
  double[] getVelX();
  void setVelX(double[] value);
  double[] getPosX();
  void setPosX(double[] value);
  double[] getPosY();
  void setPosY(double[] value);
  double[] getExistProb();
  void setExistProb(double[] value);
  boolean[] getValid();
  void setValid(boolean[] value);
}
