package sensor_fusion;

public interface associated_radar_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_fusion/associated_radar_msg";
  static final java.lang.String _DEFINITION = "radar_object_data obj \nuint64 obj_id\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  sensor_fusion.radar_object_data getObj();
  void setObj(sensor_fusion.radar_object_data value);
  long getObjId();
  void setObjId(long value);
}
