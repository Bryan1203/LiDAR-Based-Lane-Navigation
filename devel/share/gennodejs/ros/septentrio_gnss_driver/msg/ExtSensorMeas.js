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

class ExtSensorMeas {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.n = null;
      this.sb_length = null;
      this.source = null;
      this.sensor_model = null;
      this.type = null;
      this.obs_info = null;
      this.acceleration_x = null;
      this.acceleration_y = null;
      this.acceleration_z = null;
      this.angular_rate_x = null;
      this.angular_rate_y = null;
      this.angular_rate_z = null;
      this.velocity_x = null;
      this.velocity_y = null;
      this.velocity_z = null;
      this.std_dev_x = null;
      this.std_dev_y = null;
      this.std_dev_z = null;
      this.sensor_temperature = null;
      this.zero_velocity_flag = null;
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
      if (initObj.hasOwnProperty('n')) {
        this.n = initObj.n
      }
      else {
        this.n = 0;
      }
      if (initObj.hasOwnProperty('sb_length')) {
        this.sb_length = initObj.sb_length
      }
      else {
        this.sb_length = 0;
      }
      if (initObj.hasOwnProperty('source')) {
        this.source = initObj.source
      }
      else {
        this.source = [];
      }
      if (initObj.hasOwnProperty('sensor_model')) {
        this.sensor_model = initObj.sensor_model
      }
      else {
        this.sensor_model = [];
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = [];
      }
      if (initObj.hasOwnProperty('obs_info')) {
        this.obs_info = initObj.obs_info
      }
      else {
        this.obs_info = [];
      }
      if (initObj.hasOwnProperty('acceleration_x')) {
        this.acceleration_x = initObj.acceleration_x
      }
      else {
        this.acceleration_x = 0.0;
      }
      if (initObj.hasOwnProperty('acceleration_y')) {
        this.acceleration_y = initObj.acceleration_y
      }
      else {
        this.acceleration_y = 0.0;
      }
      if (initObj.hasOwnProperty('acceleration_z')) {
        this.acceleration_z = initObj.acceleration_z
      }
      else {
        this.acceleration_z = 0.0;
      }
      if (initObj.hasOwnProperty('angular_rate_x')) {
        this.angular_rate_x = initObj.angular_rate_x
      }
      else {
        this.angular_rate_x = 0.0;
      }
      if (initObj.hasOwnProperty('angular_rate_y')) {
        this.angular_rate_y = initObj.angular_rate_y
      }
      else {
        this.angular_rate_y = 0.0;
      }
      if (initObj.hasOwnProperty('angular_rate_z')) {
        this.angular_rate_z = initObj.angular_rate_z
      }
      else {
        this.angular_rate_z = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_x')) {
        this.velocity_x = initObj.velocity_x
      }
      else {
        this.velocity_x = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_y')) {
        this.velocity_y = initObj.velocity_y
      }
      else {
        this.velocity_y = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_z')) {
        this.velocity_z = initObj.velocity_z
      }
      else {
        this.velocity_z = 0.0;
      }
      if (initObj.hasOwnProperty('std_dev_x')) {
        this.std_dev_x = initObj.std_dev_x
      }
      else {
        this.std_dev_x = 0.0;
      }
      if (initObj.hasOwnProperty('std_dev_y')) {
        this.std_dev_y = initObj.std_dev_y
      }
      else {
        this.std_dev_y = 0.0;
      }
      if (initObj.hasOwnProperty('std_dev_z')) {
        this.std_dev_z = initObj.std_dev_z
      }
      else {
        this.std_dev_z = 0.0;
      }
      if (initObj.hasOwnProperty('sensor_temperature')) {
        this.sensor_temperature = initObj.sensor_temperature
      }
      else {
        this.sensor_temperature = 0.0;
      }
      if (initObj.hasOwnProperty('zero_velocity_flag')) {
        this.zero_velocity_flag = initObj.zero_velocity_flag
      }
      else {
        this.zero_velocity_flag = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExtSensorMeas
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [n]
    bufferOffset = _serializer.uint8(obj.n, buffer, bufferOffset);
    // Serialize message field [sb_length]
    bufferOffset = _serializer.uint8(obj.sb_length, buffer, bufferOffset);
    // Serialize message field [source]
    bufferOffset = _arraySerializer.uint8(obj.source, buffer, bufferOffset, null);
    // Serialize message field [sensor_model]
    bufferOffset = _arraySerializer.uint8(obj.sensor_model, buffer, bufferOffset, null);
    // Serialize message field [type]
    bufferOffset = _arraySerializer.uint8(obj.type, buffer, bufferOffset, null);
    // Serialize message field [obs_info]
    bufferOffset = _arraySerializer.uint8(obj.obs_info, buffer, bufferOffset, null);
    // Serialize message field [acceleration_x]
    bufferOffset = _serializer.float64(obj.acceleration_x, buffer, bufferOffset);
    // Serialize message field [acceleration_y]
    bufferOffset = _serializer.float64(obj.acceleration_y, buffer, bufferOffset);
    // Serialize message field [acceleration_z]
    bufferOffset = _serializer.float64(obj.acceleration_z, buffer, bufferOffset);
    // Serialize message field [angular_rate_x]
    bufferOffset = _serializer.float64(obj.angular_rate_x, buffer, bufferOffset);
    // Serialize message field [angular_rate_y]
    bufferOffset = _serializer.float64(obj.angular_rate_y, buffer, bufferOffset);
    // Serialize message field [angular_rate_z]
    bufferOffset = _serializer.float64(obj.angular_rate_z, buffer, bufferOffset);
    // Serialize message field [velocity_x]
    bufferOffset = _serializer.float32(obj.velocity_x, buffer, bufferOffset);
    // Serialize message field [velocity_y]
    bufferOffset = _serializer.float32(obj.velocity_y, buffer, bufferOffset);
    // Serialize message field [velocity_z]
    bufferOffset = _serializer.float32(obj.velocity_z, buffer, bufferOffset);
    // Serialize message field [std_dev_x]
    bufferOffset = _serializer.float32(obj.std_dev_x, buffer, bufferOffset);
    // Serialize message field [std_dev_y]
    bufferOffset = _serializer.float32(obj.std_dev_y, buffer, bufferOffset);
    // Serialize message field [std_dev_z]
    bufferOffset = _serializer.float32(obj.std_dev_z, buffer, bufferOffset);
    // Serialize message field [sensor_temperature]
    bufferOffset = _serializer.float32(obj.sensor_temperature, buffer, bufferOffset);
    // Serialize message field [zero_velocity_flag]
    bufferOffset = _serializer.float64(obj.zero_velocity_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExtSensorMeas
    let len;
    let data = new ExtSensorMeas(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [n]
    data.n = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sb_length]
    data.sb_length = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [source]
    data.source = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [sensor_model]
    data.sensor_model = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [type]
    data.type = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [obs_info]
    data.obs_info = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [acceleration_x]
    data.acceleration_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [acceleration_y]
    data.acceleration_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [acceleration_z]
    data.acceleration_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angular_rate_x]
    data.angular_rate_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angular_rate_y]
    data.angular_rate_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angular_rate_z]
    data.angular_rate_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [velocity_x]
    data.velocity_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity_y]
    data.velocity_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity_z]
    data.velocity_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [std_dev_x]
    data.std_dev_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [std_dev_y]
    data.std_dev_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [std_dev_z]
    data.std_dev_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sensor_temperature]
    data.sensor_temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [zero_velocity_flag]
    data.zero_velocity_flag = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.source.length;
    length += object.sensor_model.length;
    length += object.type.length;
    length += object.obs_info.length;
    return length + 117;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/ExtSensorMeas';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bc4155d6dac83ebf51ac1aba432000ae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ExtSensorMeas block 
    # Block_Number 4050
    
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader block_header
    
    uint8   n
    uint8   sb_length
    
    # ExtSensorMeasSet
    uint8[]   source
    uint8[]   sensor_model
    uint8[]   type
    uint8[]   obs_info
    
    #ExtSensorMeasAcceleration
    float64 acceleration_x # m/s^2
    float64 acceleration_y # m/s^2
    float64 acceleration_z # m/s^2
    
    #ExtSensorMeasAngularRate
    float64 angular_rate_x # deg/s
    float64 angular_rate_y # deg/s
    float64 angular_rate_z # deg/s
    
    #ExtSensorMeasVelocity
    float32 velocity_x # m/s
    float32 velocity_y # m/s
    float32 velocity_z # m/s
    float32 std_dev_x # m/s
    float32 std_dev_y # m/s
    float32 std_dev_z # m/s
    
    #ExtSensorMeasInfo
    float32 sensor_temperature # deg C
    
    #ExtSensorMeasZeroVelocityFlag
    float64 zero_velocity_flag
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
    const resolved = new ExtSensorMeas(null);
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

    if (msg.n !== undefined) {
      resolved.n = msg.n;
    }
    else {
      resolved.n = 0
    }

    if (msg.sb_length !== undefined) {
      resolved.sb_length = msg.sb_length;
    }
    else {
      resolved.sb_length = 0
    }

    if (msg.source !== undefined) {
      resolved.source = msg.source;
    }
    else {
      resolved.source = []
    }

    if (msg.sensor_model !== undefined) {
      resolved.sensor_model = msg.sensor_model;
    }
    else {
      resolved.sensor_model = []
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = []
    }

    if (msg.obs_info !== undefined) {
      resolved.obs_info = msg.obs_info;
    }
    else {
      resolved.obs_info = []
    }

    if (msg.acceleration_x !== undefined) {
      resolved.acceleration_x = msg.acceleration_x;
    }
    else {
      resolved.acceleration_x = 0.0
    }

    if (msg.acceleration_y !== undefined) {
      resolved.acceleration_y = msg.acceleration_y;
    }
    else {
      resolved.acceleration_y = 0.0
    }

    if (msg.acceleration_z !== undefined) {
      resolved.acceleration_z = msg.acceleration_z;
    }
    else {
      resolved.acceleration_z = 0.0
    }

    if (msg.angular_rate_x !== undefined) {
      resolved.angular_rate_x = msg.angular_rate_x;
    }
    else {
      resolved.angular_rate_x = 0.0
    }

    if (msg.angular_rate_y !== undefined) {
      resolved.angular_rate_y = msg.angular_rate_y;
    }
    else {
      resolved.angular_rate_y = 0.0
    }

    if (msg.angular_rate_z !== undefined) {
      resolved.angular_rate_z = msg.angular_rate_z;
    }
    else {
      resolved.angular_rate_z = 0.0
    }

    if (msg.velocity_x !== undefined) {
      resolved.velocity_x = msg.velocity_x;
    }
    else {
      resolved.velocity_x = 0.0
    }

    if (msg.velocity_y !== undefined) {
      resolved.velocity_y = msg.velocity_y;
    }
    else {
      resolved.velocity_y = 0.0
    }

    if (msg.velocity_z !== undefined) {
      resolved.velocity_z = msg.velocity_z;
    }
    else {
      resolved.velocity_z = 0.0
    }

    if (msg.std_dev_x !== undefined) {
      resolved.std_dev_x = msg.std_dev_x;
    }
    else {
      resolved.std_dev_x = 0.0
    }

    if (msg.std_dev_y !== undefined) {
      resolved.std_dev_y = msg.std_dev_y;
    }
    else {
      resolved.std_dev_y = 0.0
    }

    if (msg.std_dev_z !== undefined) {
      resolved.std_dev_z = msg.std_dev_z;
    }
    else {
      resolved.std_dev_z = 0.0
    }

    if (msg.sensor_temperature !== undefined) {
      resolved.sensor_temperature = msg.sensor_temperature;
    }
    else {
      resolved.sensor_temperature = 0.0
    }

    if (msg.zero_velocity_flag !== undefined) {
      resolved.zero_velocity_flag = msg.zero_velocity_flag;
    }
    else {
      resolved.zero_velocity_flag = 0.0
    }

    return resolved;
    }
};

module.exports = ExtSensorMeas;
