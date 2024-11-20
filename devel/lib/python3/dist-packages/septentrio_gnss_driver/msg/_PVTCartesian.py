# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from septentrio_gnss_driver/PVTCartesian.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import septentrio_gnss_driver.msg
import std_msgs.msg

class PVTCartesian(genpy.Message):
  _md5sum = "98b7f07c88704d34a9797ee6019c2d54"
  _type = "septentrio_gnss_driver/PVTCartesian"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """# PVTCartesian block
# ROS message header
std_msgs/Header header

# SBF block header including time header
BlockHeader  block_header   

uint8        mode
uint8        error        
float64      x             # m    
float64      y             # m             
float64      z             # m      
float32      undulation    # m 
float32      vx            # m/s         
float32      vy            # m/s 
float32      vz            # m/s
float32      cog           # deg
float64      rx_clk_bias   # ms
float32      rx_clk_drift  # ppm
uint8        time_system 
uint8        datum
uint8        nr_sv 
uint8        wa_corr_info
uint16       reference_id
uint16       mean_corr_age # 0.01s
uint32       signal_info
uint8        alert_flag
uint8        nr_bases
uint16       ppp_info      # s
uint16       latency       # 0.0001 s
uint16       h_accuracy    # 0.01 m
uint16       v_accuracy    # 0.01 m
uint8        misc

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: septentrio_gnss_driver/BlockHeader
# Blockheader including time header for all ROS messages that publish SBF blocks

uint8     sync_1
uint8     sync_2
uint16    crc
uint16    id
uint8     revision
uint16    length

uint32    tow # ms since week start
uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     """
  __slots__ = ['header','block_header','mode','error','x','y','z','undulation','vx','vy','vz','cog','rx_clk_bias','rx_clk_drift','time_system','datum','nr_sv','wa_corr_info','reference_id','mean_corr_age','signal_info','alert_flag','nr_bases','ppp_info','latency','h_accuracy','v_accuracy','misc']
  _slot_types = ['std_msgs/Header','septentrio_gnss_driver/BlockHeader','uint8','uint8','float64','float64','float64','float32','float32','float32','float32','float32','float64','float32','uint8','uint8','uint8','uint8','uint16','uint16','uint32','uint8','uint8','uint16','uint16','uint16','uint16','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,block_header,mode,error,x,y,z,undulation,vx,vy,vz,cog,rx_clk_bias,rx_clk_drift,time_system,datum,nr_sv,wa_corr_info,reference_id,mean_corr_age,signal_info,alert_flag,nr_bases,ppp_info,latency,h_accuracy,v_accuracy,misc

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PVTCartesian, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.block_header is None:
        self.block_header = septentrio_gnss_driver.msg.BlockHeader()
      if self.mode is None:
        self.mode = 0
      if self.error is None:
        self.error = 0
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.undulation is None:
        self.undulation = 0.
      if self.vx is None:
        self.vx = 0.
      if self.vy is None:
        self.vy = 0.
      if self.vz is None:
        self.vz = 0.
      if self.cog is None:
        self.cog = 0.
      if self.rx_clk_bias is None:
        self.rx_clk_bias = 0.
      if self.rx_clk_drift is None:
        self.rx_clk_drift = 0.
      if self.time_system is None:
        self.time_system = 0
      if self.datum is None:
        self.datum = 0
      if self.nr_sv is None:
        self.nr_sv = 0
      if self.wa_corr_info is None:
        self.wa_corr_info = 0
      if self.reference_id is None:
        self.reference_id = 0
      if self.mean_corr_age is None:
        self.mean_corr_age = 0
      if self.signal_info is None:
        self.signal_info = 0
      if self.alert_flag is None:
        self.alert_flag = 0
      if self.nr_bases is None:
        self.nr_bases = 0
      if self.ppp_info is None:
        self.ppp_info = 0
      if self.latency is None:
        self.latency = 0
      if self.h_accuracy is None:
        self.h_accuracy = 0
      if self.v_accuracy is None:
        self.v_accuracy = 0
      if self.misc is None:
        self.misc = 0
    else:
      self.header = std_msgs.msg.Header()
      self.block_header = septentrio_gnss_driver.msg.BlockHeader()
      self.mode = 0
      self.error = 0
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.undulation = 0.
      self.vx = 0.
      self.vy = 0.
      self.vz = 0.
      self.cog = 0.
      self.rx_clk_bias = 0.
      self.rx_clk_drift = 0.
      self.time_system = 0
      self.datum = 0
      self.nr_sv = 0
      self.wa_corr_info = 0
      self.reference_id = 0
      self.mean_corr_age = 0
      self.signal_info = 0
      self.alert_flag = 0
      self.nr_bases = 0
      self.ppp_info = 0
      self.latency = 0
      self.h_accuracy = 0
      self.v_accuracy = 0
      self.misc = 0

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B2HBHIH2B3d5fdf4B2HI2B4HB().pack(_x.block_header.sync_1, _x.block_header.sync_2, _x.block_header.crc, _x.block_header.id, _x.block_header.revision, _x.block_header.length, _x.block_header.tow, _x.block_header.wnc, _x.mode, _x.error, _x.x, _x.y, _x.z, _x.undulation, _x.vx, _x.vy, _x.vz, _x.cog, _x.rx_clk_bias, _x.rx_clk_drift, _x.time_system, _x.datum, _x.nr_sv, _x.wa_corr_info, _x.reference_id, _x.mean_corr_age, _x.signal_info, _x.alert_flag, _x.nr_bases, _x.ppp_info, _x.latency, _x.h_accuracy, _x.v_accuracy, _x.misc))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.block_header is None:
        self.block_header = septentrio_gnss_driver.msg.BlockHeader()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 96
      (_x.block_header.sync_1, _x.block_header.sync_2, _x.block_header.crc, _x.block_header.id, _x.block_header.revision, _x.block_header.length, _x.block_header.tow, _x.block_header.wnc, _x.mode, _x.error, _x.x, _x.y, _x.z, _x.undulation, _x.vx, _x.vy, _x.vz, _x.cog, _x.rx_clk_bias, _x.rx_clk_drift, _x.time_system, _x.datum, _x.nr_sv, _x.wa_corr_info, _x.reference_id, _x.mean_corr_age, _x.signal_info, _x.alert_flag, _x.nr_bases, _x.ppp_info, _x.latency, _x.h_accuracy, _x.v_accuracy, _x.misc,) = _get_struct_2B2HBHIH2B3d5fdf4B2HI2B4HB().unpack(str[start:end])
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B2HBHIH2B3d5fdf4B2HI2B4HB().pack(_x.block_header.sync_1, _x.block_header.sync_2, _x.block_header.crc, _x.block_header.id, _x.block_header.revision, _x.block_header.length, _x.block_header.tow, _x.block_header.wnc, _x.mode, _x.error, _x.x, _x.y, _x.z, _x.undulation, _x.vx, _x.vy, _x.vz, _x.cog, _x.rx_clk_bias, _x.rx_clk_drift, _x.time_system, _x.datum, _x.nr_sv, _x.wa_corr_info, _x.reference_id, _x.mean_corr_age, _x.signal_info, _x.alert_flag, _x.nr_bases, _x.ppp_info, _x.latency, _x.h_accuracy, _x.v_accuracy, _x.misc))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.block_header is None:
        self.block_header = septentrio_gnss_driver.msg.BlockHeader()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 96
      (_x.block_header.sync_1, _x.block_header.sync_2, _x.block_header.crc, _x.block_header.id, _x.block_header.revision, _x.block_header.length, _x.block_header.tow, _x.block_header.wnc, _x.mode, _x.error, _x.x, _x.y, _x.z, _x.undulation, _x.vx, _x.vy, _x.vz, _x.cog, _x.rx_clk_bias, _x.rx_clk_drift, _x.time_system, _x.datum, _x.nr_sv, _x.wa_corr_info, _x.reference_id, _x.mean_corr_age, _x.signal_info, _x.alert_flag, _x.nr_bases, _x.ppp_info, _x.latency, _x.h_accuracy, _x.v_accuracy, _x.misc,) = _get_struct_2B2HBHIH2B3d5fdf4B2HI2B4HB().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2B2HBHIH2B3d5fdf4B2HI2B4HB = None
def _get_struct_2B2HBHIH2B3d5fdf4B2HI2B4HB():
    global _struct_2B2HBHIH2B3d5fdf4B2HI2B4HB
    if _struct_2B2HBHIH2B3d5fdf4B2HI2B4HB is None:
        _struct_2B2HBHIH2B3d5fdf4B2HI2B4HB = struct.Struct("<2B2HBHIH2B3d5fdf4B2HI2B4HB")
    return _struct_2B2HBHIH2B3d5fdf4B2HI2B4HB
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I