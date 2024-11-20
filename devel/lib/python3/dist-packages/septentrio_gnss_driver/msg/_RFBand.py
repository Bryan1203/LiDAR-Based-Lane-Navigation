# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from septentrio_gnss_driver/RFBand.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class RFBand(genpy.Message):
  _md5sum = "987cd35fc563b11daae475e3e9a37fd6"
  _type = "septentrio_gnss_driver/RFBand"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# RFband block

uint32 frequency # Hz
uint16 bandwidth # kHz
uint8  info"""
  __slots__ = ['frequency','bandwidth','info']
  _slot_types = ['uint32','uint16','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       frequency,bandwidth,info

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(RFBand, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.frequency is None:
        self.frequency = 0
      if self.bandwidth is None:
        self.bandwidth = 0
      if self.info is None:
        self.info = 0
    else:
      self.frequency = 0
      self.bandwidth = 0
      self.info = 0

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
      buff.write(_get_struct_IHB().pack(_x.frequency, _x.bandwidth, _x.info))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 7
      (_x.frequency, _x.bandwidth, _x.info,) = _get_struct_IHB().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_IHB().pack(_x.frequency, _x.bandwidth, _x.info))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 7
      (_x.frequency, _x.bandwidth, _x.info,) = _get_struct_IHB().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_IHB = None
def _get_struct_IHB():
    global _struct_IHB
    if _struct_IHB is None:
        _struct_IHB = struct.Struct("<IHB")
    return _struct_IHB