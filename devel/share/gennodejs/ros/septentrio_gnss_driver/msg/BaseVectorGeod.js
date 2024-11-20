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
let VectorInfoGeod = require('./VectorInfoGeod.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class BaseVectorGeod {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.n = null;
      this.sb_length = null;
      this.vector_info_geod = null;
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
      if (initObj.hasOwnProperty('sb_length')) {
        this.sb_length = initObj.sb_length
      }
      else {
        this.sb_length = 0;
      }
      if (initObj.hasOwnProperty('vector_info_geod')) {
        this.vector_info_geod = initObj.vector_info_geod
      }
      else {
        this.vector_info_geod = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BaseVectorGeod
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [n]
    bufferOffset = _serializer.uint8(obj.n, buffer, bufferOffset);
    // Serialize message field [sb_length]
    bufferOffset = _serializer.uint8(obj.sb_length, buffer, bufferOffset);
    // Serialize message field [vector_info_geod]
    // Serialize the length for message field [vector_info_geod]
    bufferOffset = _serializer.uint32(obj.vector_info_geod.length, buffer, bufferOffset);
    obj.vector_info_geod.forEach((val) => {
      bufferOffset = VectorInfoGeod.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BaseVectorGeod
    let len;
    let data = new BaseVectorGeod(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [n]
    data.n = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sb_length]
    data.sb_length = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vector_info_geod]
    // Deserialize array length for message field [vector_info_geod]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.vector_info_geod = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.vector_info_geod[i] = VectorInfoGeod.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 52 * object.vector_info_geod.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/BaseVectorGeod';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1349428be748ae82d66f928c5daf71dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # BaseVectorGeod block 
    # Block_Number 4028
    
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader block_header
    
    uint8   n
    uint8   sb_length
    
    VectorInfoGeod[] vector_info_geod 
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
    MSG: septentrio_gnss_driver/VectorInfoGeod
    # VectorInfoGeod block
       
    uint8   nr_sv
    uint8   error
    uint8   mode
    uint8   misc
    float64 delta_east   # m
    float64 delta_north  # m
    float64 delta_up     # m
    float32 delta_ve     # m
    float32 delta_vn     # m
    float32 delta_vu     # m
    uint16  azimuth      # 0.01 deg
    int16   elevation    # 0.01 deg
    uint16  reference_id
    uint16  corr_age     # 0.01 s
    uint32  signal_info
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BaseVectorGeod(null);
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

    if (msg.sb_length !== undefined) {
      resolved.sb_length = msg.sb_length;
    }
    else {
      resolved.sb_length = 0
    }

    if (msg.vector_info_geod !== undefined) {
      resolved.vector_info_geod = new Array(msg.vector_info_geod.length);
      for (let i = 0; i < resolved.vector_info_geod.length; ++i) {
        resolved.vector_info_geod[i] = VectorInfoGeod.Resolve(msg.vector_info_geod[i]);
      }
    }
    else {
      resolved.vector_info_geod = []
    }

    return resolved;
    }
};

module.exports = BaseVectorGeod;
