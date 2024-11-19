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

class PosCovGeodetic {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.mode = null;
      this.error = null;
      this.cov_latlat = null;
      this.cov_lonlon = null;
      this.cov_hgthgt = null;
      this.cov_bb = null;
      this.cov_latlon = null;
      this.cov_lathgt = null;
      this.cov_latb = null;
      this.cov_lonhgt = null;
      this.cov_lonb = null;
      this.cov_hb = null;
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
      if (initObj.hasOwnProperty('cov_latlat')) {
        this.cov_latlat = initObj.cov_latlat
      }
      else {
        this.cov_latlat = 0.0;
      }
      if (initObj.hasOwnProperty('cov_lonlon')) {
        this.cov_lonlon = initObj.cov_lonlon
      }
      else {
        this.cov_lonlon = 0.0;
      }
      if (initObj.hasOwnProperty('cov_hgthgt')) {
        this.cov_hgthgt = initObj.cov_hgthgt
      }
      else {
        this.cov_hgthgt = 0.0;
      }
      if (initObj.hasOwnProperty('cov_bb')) {
        this.cov_bb = initObj.cov_bb
      }
      else {
        this.cov_bb = 0.0;
      }
      if (initObj.hasOwnProperty('cov_latlon')) {
        this.cov_latlon = initObj.cov_latlon
      }
      else {
        this.cov_latlon = 0.0;
      }
      if (initObj.hasOwnProperty('cov_lathgt')) {
        this.cov_lathgt = initObj.cov_lathgt
      }
      else {
        this.cov_lathgt = 0.0;
      }
      if (initObj.hasOwnProperty('cov_latb')) {
        this.cov_latb = initObj.cov_latb
      }
      else {
        this.cov_latb = 0.0;
      }
      if (initObj.hasOwnProperty('cov_lonhgt')) {
        this.cov_lonhgt = initObj.cov_lonhgt
      }
      else {
        this.cov_lonhgt = 0.0;
      }
      if (initObj.hasOwnProperty('cov_lonb')) {
        this.cov_lonb = initObj.cov_lonb
      }
      else {
        this.cov_lonb = 0.0;
      }
      if (initObj.hasOwnProperty('cov_hb')) {
        this.cov_hb = initObj.cov_hb
      }
      else {
        this.cov_hb = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PosCovGeodetic
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    // Serialize message field [cov_latlat]
    bufferOffset = _serializer.float32(obj.cov_latlat, buffer, bufferOffset);
    // Serialize message field [cov_lonlon]
    bufferOffset = _serializer.float32(obj.cov_lonlon, buffer, bufferOffset);
    // Serialize message field [cov_hgthgt]
    bufferOffset = _serializer.float32(obj.cov_hgthgt, buffer, bufferOffset);
    // Serialize message field [cov_bb]
    bufferOffset = _serializer.float32(obj.cov_bb, buffer, bufferOffset);
    // Serialize message field [cov_latlon]
    bufferOffset = _serializer.float32(obj.cov_latlon, buffer, bufferOffset);
    // Serialize message field [cov_lathgt]
    bufferOffset = _serializer.float32(obj.cov_lathgt, buffer, bufferOffset);
    // Serialize message field [cov_latb]
    bufferOffset = _serializer.float32(obj.cov_latb, buffer, bufferOffset);
    // Serialize message field [cov_lonhgt]
    bufferOffset = _serializer.float32(obj.cov_lonhgt, buffer, bufferOffset);
    // Serialize message field [cov_lonb]
    bufferOffset = _serializer.float32(obj.cov_lonb, buffer, bufferOffset);
    // Serialize message field [cov_hb]
    bufferOffset = _serializer.float32(obj.cov_hb, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PosCovGeodetic
    let len;
    let data = new PosCovGeodetic(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cov_latlat]
    data.cov_latlat = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_lonlon]
    data.cov_lonlon = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_hgthgt]
    data.cov_hgthgt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_bb]
    data.cov_bb = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_latlon]
    data.cov_latlon = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_lathgt]
    data.cov_lathgt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_latb]
    data.cov_latb = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_lonhgt]
    data.cov_lonhgt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_lonb]
    data.cov_lonb = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_hb]
    data.cov_hb = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/PosCovGeodetic';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd5a0ad571ba33ff67cc97dcff03d6b87';
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
    float32      cov_latlat # m^2
    float32      cov_lonlon # m^2
    float32      cov_hgthgt # m^2
    float32      cov_bb     # m^2
    float32      cov_latlon # m^2
    float32      cov_lathgt # m^2
    float32      cov_latb   # m^2
    float32      cov_lonhgt # m^2
    float32      cov_lonb   # m^2
    float32      cov_hb     # m^2
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
    const resolved = new PosCovGeodetic(null);
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

    if (msg.cov_latlat !== undefined) {
      resolved.cov_latlat = msg.cov_latlat;
    }
    else {
      resolved.cov_latlat = 0.0
    }

    if (msg.cov_lonlon !== undefined) {
      resolved.cov_lonlon = msg.cov_lonlon;
    }
    else {
      resolved.cov_lonlon = 0.0
    }

    if (msg.cov_hgthgt !== undefined) {
      resolved.cov_hgthgt = msg.cov_hgthgt;
    }
    else {
      resolved.cov_hgthgt = 0.0
    }

    if (msg.cov_bb !== undefined) {
      resolved.cov_bb = msg.cov_bb;
    }
    else {
      resolved.cov_bb = 0.0
    }

    if (msg.cov_latlon !== undefined) {
      resolved.cov_latlon = msg.cov_latlon;
    }
    else {
      resolved.cov_latlon = 0.0
    }

    if (msg.cov_lathgt !== undefined) {
      resolved.cov_lathgt = msg.cov_lathgt;
    }
    else {
      resolved.cov_lathgt = 0.0
    }

    if (msg.cov_latb !== undefined) {
      resolved.cov_latb = msg.cov_latb;
    }
    else {
      resolved.cov_latb = 0.0
    }

    if (msg.cov_lonhgt !== undefined) {
      resolved.cov_lonhgt = msg.cov_lonhgt;
    }
    else {
      resolved.cov_lonhgt = 0.0
    }

    if (msg.cov_lonb !== undefined) {
      resolved.cov_lonb = msg.cov_lonb;
    }
    else {
      resolved.cov_lonb = 0.0
    }

    if (msg.cov_hb !== undefined) {
      resolved.cov_hb = msg.cov_hb;
    }
    else {
      resolved.cov_hb = 0.0
    }

    return resolved;
    }
};

module.exports = PosCovGeodetic;
