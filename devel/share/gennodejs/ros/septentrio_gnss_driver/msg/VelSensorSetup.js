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

class VelSensorSetup {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.port = null;
      this.lever_arm_x = null;
      this.lever_arm_y = null;
      this.lever_arm_z = null;
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
      if (initObj.hasOwnProperty('port')) {
        this.port = initObj.port
      }
      else {
        this.port = 0;
      }
      if (initObj.hasOwnProperty('lever_arm_x')) {
        this.lever_arm_x = initObj.lever_arm_x
      }
      else {
        this.lever_arm_x = 0.0;
      }
      if (initObj.hasOwnProperty('lever_arm_y')) {
        this.lever_arm_y = initObj.lever_arm_y
      }
      else {
        this.lever_arm_y = 0.0;
      }
      if (initObj.hasOwnProperty('lever_arm_z')) {
        this.lever_arm_z = initObj.lever_arm_z
      }
      else {
        this.lever_arm_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VelSensorSetup
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [port]
    bufferOffset = _serializer.uint8(obj.port, buffer, bufferOffset);
    // Serialize message field [lever_arm_x]
    bufferOffset = _serializer.float32(obj.lever_arm_x, buffer, bufferOffset);
    // Serialize message field [lever_arm_y]
    bufferOffset = _serializer.float32(obj.lever_arm_y, buffer, bufferOffset);
    // Serialize message field [lever_arm_z]
    bufferOffset = _serializer.float32(obj.lever_arm_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VelSensorSetup
    let len;
    let data = new VelSensorSetup(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [port]
    data.port = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lever_arm_x]
    data.lever_arm_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lever_arm_y]
    data.lever_arm_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lever_arm_z]
    data.lever_arm_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/VelSensorSetup';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'de5f86a0756c4a0797c2dda3d121932f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # VelSensorSetup block
    # Block_Number 4244
    
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader block_header
    
    uint8   port
    float32 lever_arm_x # m
    float32 lever_arm_y # m
    float32 lever_arm_z # m
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
    const resolved = new VelSensorSetup(null);
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

    if (msg.port !== undefined) {
      resolved.port = msg.port;
    }
    else {
      resolved.port = 0
    }

    if (msg.lever_arm_x !== undefined) {
      resolved.lever_arm_x = msg.lever_arm_x;
    }
    else {
      resolved.lever_arm_x = 0.0
    }

    if (msg.lever_arm_y !== undefined) {
      resolved.lever_arm_y = msg.lever_arm_y;
    }
    else {
      resolved.lever_arm_y = 0.0
    }

    if (msg.lever_arm_z !== undefined) {
      resolved.lever_arm_z = msg.lever_arm_z;
    }
    else {
      resolved.lever_arm_z = 0.0
    }

    return resolved;
    }
};

module.exports = VelSensorSetup;
