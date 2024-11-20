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
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ReceiverTime {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.utc_year = null;
      this.utc_month = null;
      this.utc_day = null;
      this.utc_hour = null;
      this.utc_min = null;
      this.utc_second = null;
      this.delta_ls = null;
      this.sync_level = null;
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
      if (initObj.hasOwnProperty('utc_year')) {
        this.utc_year = initObj.utc_year
      }
      else {
        this.utc_year = 0;
      }
      if (initObj.hasOwnProperty('utc_month')) {
        this.utc_month = initObj.utc_month
      }
      else {
        this.utc_month = 0;
      }
      if (initObj.hasOwnProperty('utc_day')) {
        this.utc_day = initObj.utc_day
      }
      else {
        this.utc_day = 0;
      }
      if (initObj.hasOwnProperty('utc_hour')) {
        this.utc_hour = initObj.utc_hour
      }
      else {
        this.utc_hour = 0;
      }
      if (initObj.hasOwnProperty('utc_min')) {
        this.utc_min = initObj.utc_min
      }
      else {
        this.utc_min = 0;
      }
      if (initObj.hasOwnProperty('utc_second')) {
        this.utc_second = initObj.utc_second
      }
      else {
        this.utc_second = 0;
      }
      if (initObj.hasOwnProperty('delta_ls')) {
        this.delta_ls = initObj.delta_ls
      }
      else {
        this.delta_ls = 0;
      }
      if (initObj.hasOwnProperty('sync_level')) {
        this.sync_level = initObj.sync_level
      }
      else {
        this.sync_level = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReceiverTime
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [utc_year]
    bufferOffset = _serializer.int8(obj.utc_year, buffer, bufferOffset);
    // Serialize message field [utc_month]
    bufferOffset = _serializer.int8(obj.utc_month, buffer, bufferOffset);
    // Serialize message field [utc_day]
    bufferOffset = _serializer.int8(obj.utc_day, buffer, bufferOffset);
    // Serialize message field [utc_hour]
    bufferOffset = _serializer.int8(obj.utc_hour, buffer, bufferOffset);
    // Serialize message field [utc_min]
    bufferOffset = _serializer.int8(obj.utc_min, buffer, bufferOffset);
    // Serialize message field [utc_second]
    bufferOffset = _serializer.int8(obj.utc_second, buffer, bufferOffset);
    // Serialize message field [delta_ls]
    bufferOffset = _serializer.int8(obj.delta_ls, buffer, bufferOffset);
    // Serialize message field [sync_level]
    bufferOffset = _serializer.uint8(obj.sync_level, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReceiverTime
    let len;
    let data = new ReceiverTime(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [utc_year]
    data.utc_year = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [utc_month]
    data.utc_month = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [utc_day]
    data.utc_day = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [utc_hour]
    data.utc_hour = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [utc_min]
    data.utc_min = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [utc_second]
    data.utc_second = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [delta_ls]
    data.delta_ls = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [sync_level]
    data.sync_level = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 23;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/ReceiverTime';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e8efd4564fa50f687df91c6000b6c23';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ReceiverTime block
    # ROS message header
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader  block_header
    
    int8  utc_year    # year
    int8  utc_month   # month
    int8  utc_day     # day
    int8  utc_hour    # hour
    int8  utc_min     # minute
    int8  utc_second # s
    int8  delta_ls   # s
    uint8 sync_level
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReceiverTime(null);
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

    if (msg.utc_year !== undefined) {
      resolved.utc_year = msg.utc_year;
    }
    else {
      resolved.utc_year = 0
    }

    if (msg.utc_month !== undefined) {
      resolved.utc_month = msg.utc_month;
    }
    else {
      resolved.utc_month = 0
    }

    if (msg.utc_day !== undefined) {
      resolved.utc_day = msg.utc_day;
    }
    else {
      resolved.utc_day = 0
    }

    if (msg.utc_hour !== undefined) {
      resolved.utc_hour = msg.utc_hour;
    }
    else {
      resolved.utc_hour = 0
    }

    if (msg.utc_min !== undefined) {
      resolved.utc_min = msg.utc_min;
    }
    else {
      resolved.utc_min = 0
    }

    if (msg.utc_second !== undefined) {
      resolved.utc_second = msg.utc_second;
    }
    else {
      resolved.utc_second = 0
    }

    if (msg.delta_ls !== undefined) {
      resolved.delta_ls = msg.delta_ls;
    }
    else {
      resolved.delta_ls = 0
    }

    if (msg.sync_level !== undefined) {
      resolved.sync_level = msg.sync_level;
    }
    else {
      resolved.sync_level = 0
    }

    return resolved;
    }
};

module.exports = ReceiverTime;
