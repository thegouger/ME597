"""autogenerated by genpy from clearpath_horizon/DifferentialControl.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class DifferentialControl(genpy.Message):
  _md5sum = "7fc8e33b74ccc82007a751ce82aba911"
  _type = "clearpath_horizon/DifferentialControl"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
float64 left_p
float64 left_i
float64 left_d
float64 left_ff
float64 left_sc
float64 left_il
float64 right_p
float64 right_i
float64 right_d
float64 right_ff
float64 right_sc
float64 right_il

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

"""
  __slots__ = ['header','left_p','left_i','left_d','left_ff','left_sc','left_il','right_p','right_i','right_d','right_ff','right_sc','right_il']
  _slot_types = ['std_msgs/Header','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,left_p,left_i,left_d,left_ff,left_sc,left_il,right_p,right_i,right_d,right_ff,right_sc,right_il

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(DifferentialControl, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.left_p is None:
        self.left_p = 0.
      if self.left_i is None:
        self.left_i = 0.
      if self.left_d is None:
        self.left_d = 0.
      if self.left_ff is None:
        self.left_ff = 0.
      if self.left_sc is None:
        self.left_sc = 0.
      if self.left_il is None:
        self.left_il = 0.
      if self.right_p is None:
        self.right_p = 0.
      if self.right_i is None:
        self.right_i = 0.
      if self.right_d is None:
        self.right_d = 0.
      if self.right_ff is None:
        self.right_ff = 0.
      if self.right_sc is None:
        self.right_sc = 0.
      if self.right_il is None:
        self.right_il = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.left_p = 0.
      self.left_i = 0.
      self.left_d = 0.
      self.left_ff = 0.
      self.left_sc = 0.
      self.left_il = 0.
      self.right_p = 0.
      self.right_i = 0.
      self.right_d = 0.
      self.right_ff = 0.
      self.right_sc = 0.
      self.right_il = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_12d.pack(_x.left_p, _x.left_i, _x.left_d, _x.left_ff, _x.left_sc, _x.left_il, _x.right_p, _x.right_i, _x.right_d, _x.right_ff, _x.right_sc, _x.right_il))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 96
      (_x.left_p, _x.left_i, _x.left_d, _x.left_ff, _x.left_sc, _x.left_il, _x.right_p, _x.right_i, _x.right_d, _x.right_ff, _x.right_sc, _x.right_il,) = _struct_12d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_12d.pack(_x.left_p, _x.left_i, _x.left_d, _x.left_ff, _x.left_sc, _x.left_il, _x.right_p, _x.right_i, _x.right_d, _x.right_ff, _x.right_sc, _x.right_il))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 96
      (_x.left_p, _x.left_i, _x.left_d, _x.left_ff, _x.left_sc, _x.left_il, _x.right_p, _x.right_i, _x.right_d, _x.right_ff, _x.right_sc, _x.right_il,) = _struct_12d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_12d = struct.Struct("<12d")
