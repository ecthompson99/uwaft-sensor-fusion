package kalman_filter;

public interface object_deletion_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kalman_filter/object_deletion_msg";
  static final java.lang.String _DEFINITION = "uint16 obj_id\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  short getObjId();
  void setObjId(short value);
}
