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

class GALAuthStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.osnma_status = null;
      this.trusted_time_delta = null;
      this.gal_active_mask = null;
      this.gal_authentic_mask = null;
      this.gps_active_mask = null;
      this.gps_authentic_mask = null;
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
      if (initObj.hasOwnProperty('osnma_status')) {
        this.osnma_status = initObj.osnma_status
      }
      else {
        this.osnma_status = 0;
      }
      if (initObj.hasOwnProperty('trusted_time_delta')) {
        this.trusted_time_delta = initObj.trusted_time_delta
      }
      else {
        this.trusted_time_delta = 0.0;
      }
      if (initObj.hasOwnProperty('gal_active_mask')) {
        this.gal_active_mask = initObj.gal_active_mask
      }
      else {
        this.gal_active_mask = 0;
      }
      if (initObj.hasOwnProperty('gal_authentic_mask')) {
        this.gal_authentic_mask = initObj.gal_authentic_mask
      }
      else {
        this.gal_authentic_mask = 0;
      }
      if (initObj.hasOwnProperty('gps_active_mask')) {
        this.gps_active_mask = initObj.gps_active_mask
      }
      else {
        this.gps_active_mask = 0;
      }
      if (initObj.hasOwnProperty('gps_authentic_mask')) {
        this.gps_authentic_mask = initObj.gps_authentic_mask
      }
      else {
        this.gps_authentic_mask = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GALAuthStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [osnma_status]
    bufferOffset = _serializer.uint16(obj.osnma_status, buffer, bufferOffset);
    // Serialize message field [trusted_time_delta]
    bufferOffset = _serializer.float32(obj.trusted_time_delta, buffer, bufferOffset);
    // Serialize message field [gal_active_mask]
    bufferOffset = _serializer.uint64(obj.gal_active_mask, buffer, bufferOffset);
    // Serialize message field [gal_authentic_mask]
    bufferOffset = _serializer.uint64(obj.gal_authentic_mask, buffer, bufferOffset);
    // Serialize message field [gps_active_mask]
    bufferOffset = _serializer.uint64(obj.gps_active_mask, buffer, bufferOffset);
    // Serialize message field [gps_authentic_mask]
    bufferOffset = _serializer.uint64(obj.gps_authentic_mask, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GALAuthStatus
    let len;
    let data = new GALAuthStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [osnma_status]
    data.osnma_status = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [trusted_time_delta]
    data.trusted_time_delta = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gal_active_mask]
    data.gal_active_mask = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [gal_authentic_mask]
    data.gal_authentic_mask = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [gps_active_mask]
    data.gps_active_mask = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [gps_authentic_mask]
    data.gps_authentic_mask = _deserializer.uint64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 53;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/GALAuthStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9b9a0d56668644a236059d089716150f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # GALAuthStatus block 
    # Block_Number 4245
    
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader block_header
    
    uint16  osnma_status
    float32 trusted_time_delta # s
    uint64  gal_active_mask
    uint64  gal_authentic_mask
    uint64  gps_active_mask
    uint64  gps_authentic_mask
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
    const resolved = new GALAuthStatus(null);
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

    if (msg.osnma_status !== undefined) {
      resolved.osnma_status = msg.osnma_status;
    }
    else {
      resolved.osnma_status = 0
    }

    if (msg.trusted_time_delta !== undefined) {
      resolved.trusted_time_delta = msg.trusted_time_delta;
    }
    else {
      resolved.trusted_time_delta = 0.0
    }

    if (msg.gal_active_mask !== undefined) {
      resolved.gal_active_mask = msg.gal_active_mask;
    }
    else {
      resolved.gal_active_mask = 0
    }

    if (msg.gal_authentic_mask !== undefined) {
      resolved.gal_authentic_mask = msg.gal_authentic_mask;
    }
    else {
      resolved.gal_authentic_mask = 0
    }

    if (msg.gps_active_mask !== undefined) {
      resolved.gps_active_mask = msg.gps_active_mask;
    }
    else {
      resolved.gps_active_mask = 0
    }

    if (msg.gps_authentic_mask !== undefined) {
      resolved.gps_authentic_mask = msg.gps_authentic_mask;
    }
    else {
      resolved.gps_authentic_mask = 0
    }

    return resolved;
    }
};

module.exports = GALAuthStatus;
