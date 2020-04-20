package ecmc;

public interface sensor_diagnostic_flag_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ecmc/sensor_diagnostic_flag_msg";
  static final java.lang.String _DEFINITION = "Header header\n\nuint8[6] radar_reliability\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  org.jboss.netty.buffer.ChannelBuffer getRadarReliability();
  void setRadarReliability(org.jboss.netty.buffer.ChannelBuffer value);
}
