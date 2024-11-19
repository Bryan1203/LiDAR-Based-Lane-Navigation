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

class IMUSetup {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.serial_port = null;
      this.ant_lever_arm_x = null;
      this.ant_lever_arm_y = null;
      this.ant_lever_arm_z = null;
      this.theta_x = null;
      this.theta_y = null;
      this.theta_z = null;
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
      if (initObj.hasOwnProperty('serial_port')) {
        this.serial_port = initObj.serial_port
      }
      else {
        this.serial_port = 0;
      }
      if (initObj.hasOwnProperty('ant_lever_arm_x')) {
        this.ant_lever_arm_x = initObj.ant_lever_arm_x
      }
      else {
        this.ant_lever_arm_x = 0.0;
      }
      if (initObj.hasOwnProperty('ant_lever_arm_y')) {
        this.ant_lever_arm_y = initObj.ant_lever_arm_y
      }
      else {
        this.ant_lever_arm_y = 0.0;
      }
      if (initObj.hasOwnProperty('ant_lever_arm_z')) {
        this.ant_lever_arm_z = initObj.ant_lever_arm_z
      }
      else {
        this.ant_lever_arm_z = 0.0;
      }
      if (initObj.hasOwnProperty('theta_x')) {
        this.theta_x = initObj.theta_x
      }
      else {
        this.theta_x = 0.0;
      }
      if (initObj.hasOwnProperty('theta_y')) {
        this.theta_y = initObj.theta_y
      }
      else {
        this.theta_y = 0.0;
      }
      if (initObj.hasOwnProperty('theta_z')) {
        this.theta_z = initObj.theta_z
      }
      else {
        this.theta_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IMUSetup
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [serial_port]
    bufferOffset = _serializer.uint8(obj.serial_port, buffer, bufferOffset);
    // Serialize message field [ant_lever_arm_x]
    bufferOffset = _serializer.float32(obj.ant_lever_arm_x, buffer, bufferOffset);
    // Serialize message field [ant_lever_arm_y]
    bufferOffset = _serializer.float32(obj.ant_lever_arm_y, buffer, bufferOffset);
    // Serialize message field [ant_lever_arm_z]
    bufferOffset = _serializer.float32(obj.ant_lever_arm_z, buffer, bufferOffset);
    // Serialize message field [theta_x]
    bufferOffset = _serializer.float32(obj.theta_x, buffer, bufferOffset);
    // Serialize message field [theta_y]
    bufferOffset = _serializer.float32(obj.theta_y, buffer, bufferOffset);
    // Serialize message field [theta_z]
    bufferOffset = _serializer.float32(obj.theta_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IMUSetup
    let len;
    let data = new IMUSetup(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [serial_port]
    data.serial_port = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [ant_lever_arm_x]
    data.ant_lever_arm_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ant_lever_arm_y]
    data.ant_lever_arm_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ant_lever_arm_z]
    data.ant_lever_arm_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta_x]
    data.theta_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta_y]
    data.theta_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta_z]
    data.theta_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/IMUSetup';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'da16656e2ba26ce4129ef1261f4a5033';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # IMUSetup block
    # Block_Number 4224
    
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader block_header 
    
    uint8   serial_port
    float32 ant_lever_arm_x # m
    float32 ant_lever_arm_y # m
    float32 ant_lever_arm_z # m
    float32 theta_x # deg
    float32 theta_y # deg
    float32 theta_z # deg
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
    const resolved = new IMUSetup(null);
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

    if (msg.serial_port !== undefined) {
      resolved.serial_port = msg.serial_port;
    }
    else {
      resolved.serial_port = 0
    }

    if (msg.ant_lever_arm_x !== undefined) {
      resolved.ant_lever_arm_x = msg.ant_lever_arm_x;
    }
    else {
      resolved.ant_lever_arm_x = 0.0
    }

    if (msg.ant_lever_arm_y !== undefined) {
      resolved.ant_lever_arm_y = msg.ant_lever_arm_y;
    }
    else {
      resolved.ant_lever_arm_y = 0.0
    }

    if (msg.ant_lever_arm_z !== undefined) {
      resolved.ant_lever_arm_z = msg.ant_lever_arm_z;
    }
    else {
      resolved.ant_lever_arm_z = 0.0
    }

    if (msg.theta_x !== undefined) {
      resolved.theta_x = msg.theta_x;
    }
    else {
      resolved.theta_x = 0.0
    }

    if (msg.theta_y !== undefined) {
      resolved.theta_y = msg.theta_y;
    }
    else {
      resolved.theta_y = 0.0
    }

    if (msg.theta_z !== undefined) {
      resolved.theta_z = msg.theta_z;
    }
    else {
      resolved.theta_z = 0.0
    }

    return resolved;
    }
};

module.exports = IMUSetup;
