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

class AttCovEuler {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.error = null;
      this.cov_headhead = null;
      this.cov_pitchpitch = null;
      this.cov_rollroll = null;
      this.cov_headpitch = null;
      this.cov_headroll = null;
      this.cov_pitchroll = null;
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
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = 0;
      }
      if (initObj.hasOwnProperty('cov_headhead')) {
        this.cov_headhead = initObj.cov_headhead
      }
      else {
        this.cov_headhead = 0.0;
      }
      if (initObj.hasOwnProperty('cov_pitchpitch')) {
        this.cov_pitchpitch = initObj.cov_pitchpitch
      }
      else {
        this.cov_pitchpitch = 0.0;
      }
      if (initObj.hasOwnProperty('cov_rollroll')) {
        this.cov_rollroll = initObj.cov_rollroll
      }
      else {
        this.cov_rollroll = 0.0;
      }
      if (initObj.hasOwnProperty('cov_headpitch')) {
        this.cov_headpitch = initObj.cov_headpitch
      }
      else {
        this.cov_headpitch = 0.0;
      }
      if (initObj.hasOwnProperty('cov_headroll')) {
        this.cov_headroll = initObj.cov_headroll
      }
      else {
        this.cov_headroll = 0.0;
      }
      if (initObj.hasOwnProperty('cov_pitchroll')) {
        this.cov_pitchroll = initObj.cov_pitchroll
      }
      else {
        this.cov_pitchroll = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AttCovEuler
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    // Serialize message field [cov_headhead]
    bufferOffset = _serializer.float32(obj.cov_headhead, buffer, bufferOffset);
    // Serialize message field [cov_pitchpitch]
    bufferOffset = _serializer.float32(obj.cov_pitchpitch, buffer, bufferOffset);
    // Serialize message field [cov_rollroll]
    bufferOffset = _serializer.float32(obj.cov_rollroll, buffer, bufferOffset);
    // Serialize message field [cov_headpitch]
    bufferOffset = _serializer.float32(obj.cov_headpitch, buffer, bufferOffset);
    // Serialize message field [cov_headroll]
    bufferOffset = _serializer.float32(obj.cov_headroll, buffer, bufferOffset);
    // Serialize message field [cov_pitchroll]
    bufferOffset = _serializer.float32(obj.cov_pitchroll, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AttCovEuler
    let len;
    let data = new AttCovEuler(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cov_headhead]
    data.cov_headhead = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_pitchpitch]
    data.cov_pitchpitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_rollroll]
    data.cov_rollroll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_headpitch]
    data.cov_headpitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_headroll]
    data.cov_headroll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_pitchroll]
    data.cov_pitchroll = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/AttCovEuler';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e0afc6c2dfb1f98f719a573ace215ea7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # AttCovEuler block
    # ROS message header
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader  block_header   
    
    uint8        error   
    float32      cov_headhead   # deg^2
    float32      cov_pitchpitch # deg^2
    float32      cov_rollroll   # deg^2 
    float32      cov_headpitch  # deg^2
    float32      cov_headroll   # deg^2
    float32      cov_pitchroll  # deg^2
    
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
    const resolved = new AttCovEuler(null);
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

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = 0
    }

    if (msg.cov_headhead !== undefined) {
      resolved.cov_headhead = msg.cov_headhead;
    }
    else {
      resolved.cov_headhead = 0.0
    }

    if (msg.cov_pitchpitch !== undefined) {
      resolved.cov_pitchpitch = msg.cov_pitchpitch;
    }
    else {
      resolved.cov_pitchpitch = 0.0
    }

    if (msg.cov_rollroll !== undefined) {
      resolved.cov_rollroll = msg.cov_rollroll;
    }
    else {
      resolved.cov_rollroll = 0.0
    }

    if (msg.cov_headpitch !== undefined) {
      resolved.cov_headpitch = msg.cov_headpitch;
    }
    else {
      resolved.cov_headpitch = 0.0
    }

    if (msg.cov_headroll !== undefined) {
      resolved.cov_headroll = msg.cov_headroll;
    }
    else {
      resolved.cov_headroll = 0.0
    }

    if (msg.cov_pitchroll !== undefined) {
      resolved.cov_pitchroll = msg.cov_pitchroll;
    }
    else {
      resolved.cov_pitchroll = 0.0
    }

    return resolved;
    }
};

module.exports = AttCovEuler;
