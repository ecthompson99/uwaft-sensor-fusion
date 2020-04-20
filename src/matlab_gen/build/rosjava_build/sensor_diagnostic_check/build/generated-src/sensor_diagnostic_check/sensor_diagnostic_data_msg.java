package sensor_diagnostic_check;

public interface sensor_diagnostic_data_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_diagnostic_check/sensor_diagnostic_data_msg";
  static final java.lang.String _DEFINITION = "Header header\n\nfloat64 starter_consistency\n\nfloat64 time_stamp\nfloat64 ender_consistency\nfloat64 counter\nfloat64 check_sum\n\nuint16 itc_info\nbool hardware_fail\nbool sgu_fail\nfloat64 horizontal_misalign\nfloat64 absorb_blind\nfloat64 distort_blind\nuint8 message_counter\nuint32 message_crc\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getStarterConsistency();
  void setStarterConsistency(double value);
  double getTimeStamp();
  void setTimeStamp(double value);
  double getEnderConsistency();
  void setEnderConsistency(double value);
  double getCounter();
  void setCounter(double value);
  double getCheckSum();
  void setCheckSum(double value);
  short getItcInfo();
  void setItcInfo(short value);
  boolean getHardwareFail();
  void setHardwareFail(boolean value);
  boolean getSguFail();
  void setSguFail(boolean value);
  double getHorizontalMisalign();
  void setHorizontalMisalign(double value);
  double getAbsorbBlind();
  void setAbsorbBlind(double value);
  double getDistortBlind();
  void setDistortBlind(double value);
  byte getMessageCounter();
  void setMessageCounter(byte value);
  int getMessageCrc();
  void setMessageCrc(int value);
}
