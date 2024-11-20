// Auto-generated. Do not edit!

// (in-package septentrio_gnss_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BlockHeader = require('./BlockHeader.js');
let MeasEpochChannelType1 = require('./MeasEpochChannelType1.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MeasEpoch {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.n = null;
      this.sb1_length = null;
      this.sb2_length = null;
      this.common_flags = null;
      this.cum_clk_jumps = null;
      this.type1 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('block_header')) {
        this.block_header = initObj.block_header
      }
      else {
        this.block_header = new BlockHeader();
      }
      if (initObj.hasOwnProperty('n')) {
        this.n = initObj.n
      }
      else {
        this.n = 0;
      }
      if (initObj.hasOwnProperty('sb1_length')) {
        this.sb1_length = initObj.sb1_length
      }
      else {
        this.sb1_length = 0;
      }
      if (initObj.hasOwnProperty('sb2_length')) {
        this.sb2_length = initObj.sb2_length
      }
      else {
        this.sb2_length = 0;
      }
      if (initObj.hasOwnProperty('common_flags')) {
        this.common_flags = initObj.common_flags
      }
      else {
        this.common_flags = 0;
      }
      if (initObj.hasOwnProperty('cum_clk_jumps')) {
        this.cum_clk_jumps = initObj.cum_clk_jumps
      }
      else {
        this.cum_clk_jumps = 0;
      }
      if (initObj.hasOwnProperty('type1')) {
        this.type1 = initObj.type1
      }
      else {
        this.type1 = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MeasEpoch
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [n]
    bufferOffset = _serializer.uint8(obj.n, buffer, bufferOffset);
    // Serialize message field [sb1_length]
    bufferOffset = _serializer.uint8(obj.sb1_length, buffer, bufferOffset);
    // Serialize message field [sb2_length]
    bufferOffset = _serializer.uint8(obj.sb2_length, buffer, bufferOffset);
    // Serialize message field [common_flags]
    bufferOffset = _serializer.uint8(obj.common_flags, buffer, bufferOffset);
    // Serialize message field [cum_clk_jumps]
    bufferOffset = _serializer.uint8(obj.cum_clk_jumps, buffer, bufferOffset);
    // Serialize message field [type1]
    // Serialize the length for message field [type1]
    bufferOffset = _serializer.uint32(obj.type1.length, buffer, bufferOffset);
    obj.type1.forEach((val) => {
      bufferOffset = MeasEpochChannelType1.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MeasEpoch
    let len;
    let data = new MeasEpoch(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [n]
    data.n = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sb1_length]
    data.sb1_length = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sb2_length]
    data.sb2_length = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [common_flags]
    data.common_flags = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cum_clk_jumps]
    data.cum_clk_jumps = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [type1]
    // Deserialize array length for message field [type1]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.type1 = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.type1[i] = MeasEpochChannelType1.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.type1.forEach((val) => {
      length += MeasEpochChannelType1.getMessageSize(val);
    });
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/MeasEpoch';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4cf9a1d78edfe10d2a772f4a50dd0645';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MeasEpoch block
    # ROS message header
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader  block_header   
    
    uint8 n
    uint8 sb1_length
    uint8 sb2_length      
    uint8 common_flags
    uint8 cum_clk_jumps # 0.001 s
    
    MeasEpochChannelType1[] type1
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
    uint16    wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     
    ================================================================================
    MSG: septentrio_gnss_driver/MeasEpochChannelType1
    # MeasEpochChannelType1 block
       
    uint8  rx_channel
    uint8  type
    uint8  sv_id
    uint8  misc        # 4294967.296 m
    uint32 code_lsb    # 0.001 m
    int32  doppler     # 0.0001 Hz
    uint16 carrier_lsb # 0.001 cycles
    int8   carrier_msb # 65.536 cycles
    uint8  cn0         # 0.25 dB-Hz
    uint16 lock_time
    uint8  obs_info
    uint8  n2
    
    MeasEpochChannelType2[] type2
    ================================================================================
    MSG: septentrio_gnss_driver/MeasEpochChannelType2
    # MeasEpochChannelType2 block
       
    uint8  type
    uint8  lock_time          # s
    uint8  cn0                # 0.25 dB-Hz
    uint8  offsets_msb        # 65.536 m or 65.536 Hz
    int8   carrier_msb        # 65.536 cycles
    uint8  obs_info
    uint16 code_offset_lsb    # 0.001 m
    uint16 carrier_lsb        # 0.001 cycles
    uint16 doppler_offset_lsb # 0.0001 Hz
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MeasEpoch(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.block_header !== undefined) {
      resolved.block_header = BlockHeader.Resolve(msg.block_header)
    }
    else {
      resolved.block_header = new BlockHeader()
    }

    if (msg.n !== undefined) {
      resolved.n = msg.n;
    }
    else {
      resolved.n = 0
    }

    if (msg.sb1_length !== undefined) {
      resolved.sb1_length = msg.sb1_length;
    }
    else {
      resolved.sb1_length = 0
    }

    if (msg.sb2_length !== undefined) {
      resolved.sb2_length = msg.sb2_length;
    }
    else {
      resolved.sb2_length = 0
    }

    if (msg.common_flags !== undefined) {
      resolved.common_flags = msg.common_flags;
    }
    else {
      resolved.common_flags = 0
    }

    if (msg.cum_clk_jumps !== undefined) {
      resolved.cum_clk_jumps = msg.cum_clk_jumps;
    }
    else {
      resolved.cum_clk_jumps = 0
    }

    if (msg.type1 !== undefined) {
      resolved.type1 = new Array(msg.type1.length);
      for (let i = 0; i < resolved.type1.length; ++i) {
        resolved.type1[i] = MeasEpochChannelType1.Resolve(msg.type1[i]);
      }
    }
    else {
      resolved.type1 = []
    }

    return resolved;
    }
};

module.exports = MeasEpoch;
