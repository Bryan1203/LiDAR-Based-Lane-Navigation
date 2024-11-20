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

class AttEuler {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.nr_sv = null;
      this.error = null;
      this.mode = null;
      this.heading = null;
      this.pitch = null;
      this.roll = null;
      this.pitch_dot = null;
      this.roll_dot = null;
      this.heading_dot = null;
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
      if (initObj.hasOwnProperty('nr_sv')) {
        this.nr_sv = initObj.nr_sv
      }
      else {
        this.nr_sv = 0;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('heading')) {
        this.heading = initObj.heading
      }
      else {
        this.heading = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_dot')) {
        this.pitch_dot = initObj.pitch_dot
      }
      else {
        this.pitch_dot = 0.0;
      }
      if (initObj.hasOwnProperty('roll_dot')) {
        this.roll_dot = initObj.roll_dot
      }
      else {
        this.roll_dot = 0.0;
      }
      if (initObj.hasOwnProperty('heading_dot')) {
        this.heading_dot = initObj.heading_dot
      }
      else {
        this.heading_dot = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AttEuler
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [nr_sv]
    bufferOffset = _serializer.uint8(obj.nr_sv, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint16(obj.mode, buffer, bufferOffset);
    // Serialize message field [heading]
    bufferOffset = _serializer.float32(obj.heading, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float32(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch_dot]
    bufferOffset = _serializer.float32(obj.pitch_dot, buffer, bufferOffset);
    // Serialize message field [roll_dot]
    bufferOffset = _serializer.float32(obj.roll_dot, buffer, bufferOffset);
    // Serialize message field [heading_dot]
    bufferOffset = _serializer.float32(obj.heading_dot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AttEuler
    let len;
    let data = new AttEuler(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [nr_sv]
    data.nr_sv = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [heading]
    data.heading = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_dot]
    data.pitch_dot = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_dot]
    data.roll_dot = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_dot]
    data.heading_dot = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 43;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/AttEuler';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '99d953843f049fd84269fcb833dbdaa6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # AttEuler block
    # ROS message header
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader  block_header   
    
    uint8        nr_sv
    uint8        error        
    uint16       mode
    float32      heading     # deg
    float32      pitch       # deg
    float32      roll        # deg
    float32      pitch_dot   # deg/s
    float32      roll_dot    # deg/s
    float32      heading_dot # deg/s
    
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
    const resolved = new AttEuler(null);
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

    if (msg.nr_sv !== undefined) {
      resolved.nr_sv = msg.nr_sv;
    }
    else {
      resolved.nr_sv = 0
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.heading !== undefined) {
      resolved.heading = msg.heading;
    }
    else {
      resolved.heading = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch_dot !== undefined) {
      resolved.pitch_dot = msg.pitch_dot;
    }
    else {
      resolved.pitch_dot = 0.0
    }

    if (msg.roll_dot !== undefined) {
      resolved.roll_dot = msg.roll_dot;
    }
    else {
      resolved.roll_dot = 0.0
    }

    if (msg.heading_dot !== undefined) {
      resolved.heading_dot = msg.heading_dot;
    }
    else {
      resolved.heading_dot = 0.0
    }

    return resolved;
    }
};

module.exports = AttEuler;
