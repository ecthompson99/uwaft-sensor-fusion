package can_tx_rx;

public interface sensor_diagnostic_data_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "can_tx_rx/sensor_diagnostic_data_msg";
  static final java.lang.String _DEFINITION = "Header header\n\nuint8[] radar_diag_input\nuint8[] radar_info\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  org.jboss.netty.buffer.ChannelBuffer getRadarDiagInput();
  void setRadarDiagInput(org.jboss.netty.buffer.ChannelBuffer value);
  org.jboss.netty.buffer.ChannelBuffer getRadarInfo();
  void setRadarInfo(org.jboss.netty.buffer.ChannelBuffer value);
}
