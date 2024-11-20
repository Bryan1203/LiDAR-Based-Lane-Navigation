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

class VelCovCartesian {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.mode = null;
      this.error = null;
      this.cov_vxvx = null;
      this.cov_vyvy = null;
      this.cov_vzvz = null;
      this.cov_dtdt = null;
      this.cov_vxvy = null;
      this.cov_vxvz = null;
      this.cov_vxdt = null;
      this.cov_vyvz = null;
      this.cov_vydt = null;
      this.cov_vzdt = null;
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
      if (initObj.hasOwnProperty('cov_vxvx')) {
        this.cov_vxvx = initObj.cov_vxvx
      }
      else {
        this.cov_vxvx = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vyvy')) {
        this.cov_vyvy = initObj.cov_vyvy
      }
      else {
        this.cov_vyvy = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vzvz')) {
        this.cov_vzvz = initObj.cov_vzvz
      }
      else {
        this.cov_vzvz = 0.0;
      }
      if (initObj.hasOwnProperty('cov_dtdt')) {
        this.cov_dtdt = initObj.cov_dtdt
      }
      else {
        this.cov_dtdt = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vxvy')) {
        this.cov_vxvy = initObj.cov_vxvy
      }
      else {
        this.cov_vxvy = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vxvz')) {
        this.cov_vxvz = initObj.cov_vxvz
      }
      else {
        this.cov_vxvz = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vxdt')) {
        this.cov_vxdt = initObj.cov_vxdt
      }
      else {
        this.cov_vxdt = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vyvz')) {
        this.cov_vyvz = initObj.cov_vyvz
      }
      else {
        this.cov_vyvz = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vydt')) {
        this.cov_vydt = initObj.cov_vydt
      }
      else {
        this.cov_vydt = 0.0;
      }
      if (initObj.hasOwnProperty('cov_vzdt')) {
        this.cov_vzdt = initObj.cov_vzdt
      }
      else {
        this.cov_vzdt = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VelCovCartesian
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    // Serialize message field [cov_vxvx]
    bufferOffset = _serializer.float32(obj.cov_vxvx, buffer, bufferOffset);
    // Serialize message field [cov_vyvy]
    bufferOffset = _serializer.float32(obj.cov_vyvy, buffer, bufferOffset);
    // Serialize message field [cov_vzvz]
    bufferOffset = _serializer.float32(obj.cov_vzvz, buffer, bufferOffset);
    // Serialize message field [cov_dtdt]
    bufferOffset = _serializer.float32(obj.cov_dtdt, buffer, bufferOffset);
    // Serialize message field [cov_vxvy]
    bufferOffset = _serializer.float32(obj.cov_vxvy, buffer, bufferOffset);
    // Serialize message field [cov_vxvz]
    bufferOffset = _serializer.float32(obj.cov_vxvz, buffer, bufferOffset);
    // Serialize message field [cov_vxdt]
    bufferOffset = _serializer.float32(obj.cov_vxdt, buffer, bufferOffset);
    // Serialize message field [cov_vyvz]
    bufferOffset = _serializer.float32(obj.cov_vyvz, buffer, bufferOffset);
    // Serialize message field [cov_vydt]
    bufferOffset = _serializer.float32(obj.cov_vydt, buffer, bufferOffset);
    // Serialize message field [cov_vzdt]
    bufferOffset = _serializer.float32(obj.cov_vzdt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VelCovCartesian
    let len;
    let data = new VelCovCartesian(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cov_vxvx]
    data.cov_vxvx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vyvy]
    data.cov_vyvy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vzvz]
    data.cov_vzvz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_dtdt]
    data.cov_dtdt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vxvy]
    data.cov_vxvy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vxvz]
    data.cov_vxvz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vxdt]
    data.cov_vxdt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vyvz]
    data.cov_vyvz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vydt]
    data.cov_vydt = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cov_vzdt]
    data.cov_vzdt = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/VelCovCartesian';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0675e2a2bdddd5904100644aa4bf90a3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # VelCovCartesian block
    # ROS message header
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader  block_header   
    
    uint8        mode
    uint8        error        
    float32      cov_vxvx # m^2/s^2
    float32      cov_vyvy # m^2/s^2
    float32      cov_vzvz # m^2/s^2
    float32      cov_dtdt # m^2/s^2
    float32      cov_vxvy # m^2/s^2
    float32      cov_vxvz # m^2/s^2
    float32      cov_vxdt # m^2/s^2
    float32      cov_vyvz # m^2/s^2
    float32      cov_vydt # m^2/s^2
    float32      cov_vzdt # m^2/s^2
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
    const resolved = new VelCovCartesian(null);
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

    if (msg.cov_vxvx !== undefined) {
      resolved.cov_vxvx = msg.cov_vxvx;
    }
    else {
      resolved.cov_vxvx = 0.0
    }

    if (msg.cov_vyvy !== undefined) {
      resolved.cov_vyvy = msg.cov_vyvy;
    }
    else {
      resolved.cov_vyvy = 0.0
    }

    if (msg.cov_vzvz !== undefined) {
      resolved.cov_vzvz = msg.cov_vzvz;
    }
    else {
      resolved.cov_vzvz = 0.0
    }

    if (msg.cov_dtdt !== undefined) {
      resolved.cov_dtdt = msg.cov_dtdt;
    }
    else {
      resolved.cov_dtdt = 0.0
    }

    if (msg.cov_vxvy !== undefined) {
      resolved.cov_vxvy = msg.cov_vxvy;
    }
    else {
      resolved.cov_vxvy = 0.0
    }

    if (msg.cov_vxvz !== undefined) {
      resolved.cov_vxvz = msg.cov_vxvz;
    }
    else {
      resolved.cov_vxvz = 0.0
    }

    if (msg.cov_vxdt !== undefined) {
      resolved.cov_vxdt = msg.cov_vxdt;
    }
    else {
      resolved.cov_vxdt = 0.0
    }

    if (msg.cov_vyvz !== undefined) {
      resolved.cov_vyvz = msg.cov_vyvz;
    }
    else {
      resolved.cov_vyvz = 0.0
    }

    if (msg.cov_vydt !== undefined) {
      resolved.cov_vydt = msg.cov_vydt;
    }
    else {
      resolved.cov_vydt = 0.0
    }

    if (msg.cov_vzdt !== undefined) {
      resolved.cov_vzdt = msg.cov_vzdt;
    }
    else {
      resolved.cov_vzdt = 0.0
    }

    return resolved;
    }
};

module.exports = VelCovCartesian;
