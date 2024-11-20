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

class PosCovCartesian {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.mode = null;
      this.error = null;
      this.cov_xx = null;
      this.cov_yy = null;
      this.cov_zz = null;
      this.cov_bb = null;
      this.cov_xy = null;
      this.cov_xz = null;
      this.cov_xb = null;
      this.cov_yz = null;
      this.cov_yb = null;
      this.cov_zb = null;
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
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = 0;
      }
      if (initObj.hasOwnProperty('cov_xx')) {
        this.cov_xx = initObj.cov_xx
      }
      else {
        this.cov_xx = 0.0;
      }
      if (initObj.hasOwnProperty('cov_yy')) {
        this.cov_yy = initObj.cov_yy
      }
      else {
        this.cov_yy = 0.0;
      }
      if (initObj.hasOwnProperty('cov_zz')) {
        this.cov_zz = initObj.cov_zz
      }
      else {
        this.cov_zz = 0.0;
      }
      if (initObj.hasOwnProperty('cov_bb')) {
        this.cov_bb = initObj.cov_bb
      }
      else {
        this.cov_bb = 0.0;
      }
      if (initObj.hasOwnProperty('cov_xy')) {
        this.cov_xy = initObj.cov_xy
      }
      else {
        this.cov_xy = 0.0;
      }
      if (initObj.hasOwnProperty('cov_xz')) {
        this.cov_xz = initObj.cov_xz
      }
      else {
        this.cov_xz = 0.0;
      }
      if (initObj.hasOwnProperty('cov_xb')) {
        this.cov_xb = initObj.cov_xb
      }
      else {
        this.cov_xb = 0.0;
      }
      if (initObj.hasOwnProperty('cov_yz')) {
        this.cov_yz = initObj.cov_yz
      }
      else {
        this.cov_yz = 0.0;
      }
      if (initObj.hasOwnProperty('cov_yb')) {
        this.cov_yb = initObj.cov_yb
      }
      else {
        this.cov_yb = 0.0;
      }
      if (initObj.hasOwnProperty('cov_zb')) {
        this.cov_zb = initObj.cov_zb
      }
      else {
        this.cov_zb = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PosCovCartesian
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    // Serialize message field [cov_xx]
    bufferOffset = _serializer.float32(obj.cov_xx, buffer, bufferOffset);
    // Serialize message field [cov_yy]
    bufferOffset = _serializer.float32(obj.cov_yy, buffer, bufferOffset);
    // Serialize message field [cov_zz]
    bufferOffset = _serializer.float32(obj.cov_zz, buffer, bufferOffset);
    // Serialize message field [cov_bb]
    bufferOffset = _serializer.float32(obj.cov_bb, buffer, bufferOffset);
    // Serialize message field [cov_xy]
    bufferOffset = _serializer.float32(obj.cov_xy, buffer, bufferOffset);
    // Serialize message field [cov_xz]
    bufferOffset = _serializer.float32(obj.cov_xz, buffer, bufferOffset);
    // Serialize message field [cov_xb]
    bufferOffset = _serializer.float32(obj.cov_xb, buffer, bufferOffset);
    // Serialize message field [cov_yz]
    bufferOffset = _serializer.float32(obj.cov_yz, buffer, bufferOffset);
    // Serialize message field [cov_yb]
    bufferOffset = _serializer.float32(obj.cov_yb, buffer, bufferOffset);
    // Serialize message field [cov_zb]
    bufferOffset = _serializer.float32(obj.cov_zb, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PosCovCartesian
    let len;
    let data = new PosCovCartesian(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cov_xx]
    data.cov_xx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_yy]
    data.cov_yy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_zz]
    data.cov_zz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_bb]
    data.cov_bb = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_xy]
    data.cov_xy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_xz]
    data.cov_xz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_xb]
    data.cov_xb = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_yz]
    data.cov_yz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_yb]
    data.cov_yb = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_zb]
    data.cov_zb = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/PosCovCartesian';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f9c694efd927f0282537b2693bfa92f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # PVTGeodetic block
    # ROS message header
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader  block_header   
    
    uint8        mode
    uint8        error        
    float32      cov_xx # m^2
    float32      cov_yy # m^2
    float32      cov_zz # m^2
    float32      cov_bb # m^2
    float32      cov_xy # m^2
    float32      cov_xz # m^2
    float32      cov_xb # m^2
    float32      cov_yz # m^2
    float32      cov_yb # m^2
    float32      cov_zb # m^2
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
    const resolved = new PosCovCartesian(null);
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

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = 0
    }

    if (msg.cov_xx !== undefined) {
      resolved.cov_xx = msg.cov_xx;
    }
    else {
      resolved.cov_xx = 0.0
    }

    if (msg.cov_yy !== undefined) {
      resolved.cov_yy = msg.cov_yy;
    }
    else {
      resolved.cov_yy = 0.0
    }

    if (msg.cov_zz !== undefined) {
      resolved.cov_zz = msg.cov_zz;
    }
    else {
      resolved.cov_zz = 0.0
    }

    if (msg.cov_bb !== undefined) {
      resolved.cov_bb = msg.cov_bb;
    }
    else {
      resolved.cov_bb = 0.0
    }

    if (msg.cov_xy !== undefined) {
      resolved.cov_xy = msg.cov_xy;
    }
    else {
      resolved.cov_xy = 0.0
    }

    if (msg.cov_xz !== undefined) {
      resolved.cov_xz = msg.cov_xz;
    }
    else {
      resolved.cov_xz = 0.0
    }

    if (msg.cov_xb !== undefined) {
      resolved.cov_xb = msg.cov_xb;
    }
    else {
      resolved.cov_xb = 0.0
    }

    if (msg.cov_yz !== undefined) {
      resolved.cov_yz = msg.cov_yz;
    }
    else {
      resolved.cov_yz = 0.0
    }

    if (msg.cov_yb !== undefined) {
      resolved.cov_yb = msg.cov_yb;
    }
    else {
      resolved.cov_yb = 0.0
    }

    if (msg.cov_zb !== undefined) {
      resolved.cov_zb = msg.cov_zb;
    }
    else {
      resolved.cov_zb = 0.0
    }

    return resolved;
    }
};

module.exports = PosCovCartesian;
