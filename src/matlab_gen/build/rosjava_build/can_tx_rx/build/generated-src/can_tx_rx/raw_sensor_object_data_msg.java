package can_tx_rx;

public interface raw_sensor_object_data_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "can_tx_rx/raw_sensor_object_data_msg";
  static final java.lang.String _DEFINITION = "Header header\n\nuint8[] target_info\nuint8[] obj_info\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  org.jboss.netty.buffer.ChannelBuffer getTargetInfo();
  void setTargetInfo(org.jboss.netty.buffer.ChannelBuffer value);
  org.jboss.netty.buffer.ChannelBuffer getObjInfo();
  void setObjInfo(org.jboss.netty.buffer.ChannelBuffer value);
}
