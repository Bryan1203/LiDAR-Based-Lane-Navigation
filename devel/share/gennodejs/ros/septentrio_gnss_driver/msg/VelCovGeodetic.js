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

class VelCovGeodetic {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.mode = null;
      this.error = null;
      this.cov_vnvn = null;
      this.cov_veve = null;
      this.cov_vuvu = null;
      this.cov_dtdt = null;
      this.cov_vnve = null;
      this.cov_vnvu = null;
      this.cov_vndt = null;
      this.cov_vevu = null;
      this.cov_vedt = null;
      this.cov_vudt = null;
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
      if (initObj.hasOwnProperty('cov_vnvn')) {
        this.cov_vnvn = initObj.cov_vnvn
      }
      else {
        this.cov_vnvn = 0.0;
      }
      if (initObj.hasOwnProperty('cov_veve')) {
        this.cov_veve = initObj.cov_veve
      }
      else {
        this.cov_veve = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vuvu')) {
        this.cov_vuvu = initObj.cov_vuvu
      }
      else {
        this.cov_vuvu = 0.0;
      }
      if (initObj.hasOwnProperty('cov_dtdt')) {
        this.cov_dtdt = initObj.cov_dtdt
      }
      else {
        this.cov_dtdt = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vnve')) {
        this.cov_vnve = initObj.cov_vnve
      }
      else {
        this.cov_vnve = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vnvu')) {
        this.cov_vnvu = initObj.cov_vnvu
      }
      else {
        this.cov_vnvu = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vndt')) {
        this.cov_vndt = initObj.cov_vndt
      }
      else {
        this.cov_vndt = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vevu')) {
        this.cov_vevu = initObj.cov_vevu
      }
      else {
        this.cov_vevu = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vedt')) {
        this.cov_vedt = initObj.cov_vedt
      }
      else {
        this.cov_vedt = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vudt')) {
        this.cov_vudt = initObj.cov_vudt
      }
      else {
        this.cov_vudt = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VelCovGeodetic
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    // Serialize message field [cov_vnvn]
    bufferOffset = _serializer.float32(obj.cov_vnvn, buffer, bufferOffset);
    // Serialize message field [cov_veve]
    bufferOffset = _serializer.float32(obj.cov_veve, buffer, bufferOffset);
    // Serialize message field [cov_vuvu]
    bufferOffset = _serializer.float32(obj.cov_vuvu, buffer, bufferOffset);
    // Serialize message field [cov_dtdt]
    bufferOffset = _serializer.float32(obj.cov_dtdt, buffer, bufferOffset);
    // Serialize message field [cov_vnve]
    bufferOffset = _serializer.float32(obj.cov_vnve, buffer, bufferOffset);
    // Serialize message field [cov_vnvu]
    bufferOffset = _serializer.float32(obj.cov_vnvu, buffer, bufferOffset);
    // Serialize message field [cov_vndt]
    bufferOffset = _serializer.float32(obj.cov_vndt, buffer, bufferOffset);
    // Serialize message field [cov_vevu]
    bufferOffset = _serializer.float32(obj.cov_vevu, buffer, bufferOffset);
    // Serialize message field [cov_vedt]
    bufferOffset = _serializer.float32(obj.cov_vedt, buffer, bufferOffset);
    // Serialize message field [cov_vudt]
    bufferOffset = _serializer.float32(obj.cov_vudt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VelCovGeodetic
    let len;
    let data = new VelCovGeodetic(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cov_vnvn]
    data.cov_vnvn = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_veve]
    data.cov_veve = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vuvu]
    data.cov_vuvu = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_dtdt]
    data.cov_dtdt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vnve]
    data.cov_vnve = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vnvu]
    data.cov_vnvu = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vndt]
    data.cov_vndt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vevu]
    data.cov_vevu = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vedt]
    data.cov_vedt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vudt]
    data.cov_vudt = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/VelCovGeodetic';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b62aa4ae504c44f18ac72015246a1586';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # VelCovGeodetic block
    # ROS message header
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader  block_header   
    
    uint8        mode
    uint8        error        
    float32      cov_vnvn # m^2/s^2
    float32      cov_veve # m^2/s^2
    float32      cov_vuvu # m^2/s^2
    float32      cov_dtdt # m^2/s^2
    float32      cov_vnve # m^2/s^2
    float32      cov_vnvu # m^2/s^2
    float32      cov_vndt # m^2/s^2
    float32      cov_vevu # m^2/s^2
    float32      cov_vedt # m^2/s^2
    float32      cov_vudt # m^2/s^2
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
    const resolved = new VelCovGeodetic(null);
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

    if (msg.cov_vnvn !== undefined) {
      resolved.cov_vnvn = msg.cov_vnvn;
    }
    else {
      resolved.cov_vnvn = 0.0
    }

    if (msg.cov_veve !== undefined) {
      resolved.cov_veve = msg.cov_veve;
    }
    else {
      resolved.cov_veve = 0.0
    }

    if (msg.cov_vuvu !== undefined) {
      resolved.cov_vuvu = msg.cov_vuvu;
    }
    else {
      resolved.cov_vuvu = 0.0
    }

    if (msg.cov_dtdt !== undefined) {
      resolved.cov_dtdt = msg.cov_dtdt;
    }
    else {
      resolved.cov_dtdt = 0.0
    }

    if (msg.cov_vnve !== undefined) {
      resolved.cov_vnve = msg.cov_vnve;
    }
    else {
      resolved.cov_vnve = 0.0
    }

    if (msg.cov_vnvu !== undefined) {
      resolved.cov_vnvu = msg.cov_vnvu;
    }
    else {
      resolved.cov_vnvu = 0.0
    }

    if (msg.cov_vndt !== undefined) {
      resolved.cov_vndt = msg.cov_vndt;
    }
    else {
      resolved.cov_vndt = 0.0
    }

    if (msg.cov_vevu !== undefined) {
      resolved.cov_vevu = msg.cov_vevu;
    }
    else {
      resolved.cov_vevu = 0.0
    }

    if (msg.cov_vedt !== undefined) {
      resolved.cov_vedt = msg.cov_vedt;
    }
    else {
      resolved.cov_vedt = 0.0
    }

    if (msg.cov_vudt !== undefined) {
      resolved.cov_vudt = msg.cov_vudt;
    }
    else {
      resolved.cov_vudt = 0.0
    }

    return resolved;
    }
};

module.exports = VelCovGeodetic;
