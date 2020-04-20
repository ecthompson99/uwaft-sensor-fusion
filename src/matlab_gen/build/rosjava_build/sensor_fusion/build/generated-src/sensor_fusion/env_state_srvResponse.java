package sensor_fusion;

public interface env_state_srvResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_fusion/env_state_srvResponse";
  static final java.lang.String _DEFINITION = "uint64[] id\nfloat64[] dx\nfloat64[] dy\nfloat64[] timestamp";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  long[] getId();
  void setId(long[] value);
  double[] getDx();
  void setDx(double[] value);
  double[] getDy();
  void setDy(double[] value);
  double[] getTimestamp();
  void setTimestamp(double[] value);
}
