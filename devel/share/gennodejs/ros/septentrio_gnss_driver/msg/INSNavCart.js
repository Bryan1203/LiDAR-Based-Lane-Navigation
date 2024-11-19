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

class INSNavCart {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.gnss_mode = null;
      this.error = null;
      this.info = null;
      this.gnss_age = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.accuracy = null;
      this.latency = null;
      this.datum = null;
      this.sb_list = null;
      this.x_std_dev = null;
      this.y_std_dev = null;
      this.z_std_dev = null;
      this.xy_cov = null;
      this.xz_cov = null;
      this.yz_cov = null;
      this.heading = null;
      this.pitch = null;
      this.roll = null;
      this.heading_std_dev = null;
      this.pitch_std_dev = null;
      this.roll_std_dev = null;
      this.heading_pitch_cov = null;
      this.heading_roll_cov = null;
      this.pitch_roll_cov = null;
      this.vx = null;
      this.vy = null;
      this.vz = null;
      this.vx_std_dev = null;
      this.vy_std_dev = null;
      this.vz_std_dev = null;
      this.vx_vy_cov = null;
      this.vx_vz_cov = null;
      this.vy_vz_cov = null;
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
      if (initObj.hasOwnProperty('gnss_mode')) {
        this.gnss_mode = initObj.gnss_mode
      }
      else {
        this.gnss_mode = 0;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = 0;
      }
      if (initObj.hasOwnProperty('info')) {
        this.info = initObj.info
      }
      else {
        this.info = 0;
      }
      if (initObj.hasOwnProperty('gnss_age')) {
        this.gnss_age = initObj.gnss_age
      }
      else {
        this.gnss_age = 0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('accuracy')) {
        this.accuracy = initObj.accuracy
      }
      else {
        this.accuracy = 0;
      }
      if (initObj.hasOwnProperty('latency')) {
        this.latency = initObj.latency
      }
      else {
        this.latency = 0;
      }
      if (initObj.hasOwnProperty('datum')) {
        this.datum = initObj.datum
      }
      else {
        this.datum = 0;
      }
      if (initObj.hasOwnProperty('sb_list')) {
        this.sb_list = initObj.sb_list
      }
      else {
        this.sb_list = 0;
      }
      if (initObj.hasOwnProperty('x_std_dev')) {
        this.x_std_dev = initObj.x_std_dev
      }
      else {
        this.x_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('y_std_dev')) {
        this.y_std_dev = initObj.y_std_dev
      }
      else {
        this.y_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('z_std_dev')) {
        this.z_std_dev = initObj.z_std_dev
      }
      else {
        this.z_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('xy_cov')) {
        this.xy_cov = initObj.xy_cov
      }
      else {
        this.xy_cov = 0.0;
      }
      if (initObj.hasOwnProperty('xz_cov')) {
        this.xz_cov = initObj.xz_cov
      }
      else {
        this.xz_cov = 0.0;
      }
      if (initObj.hasOwnProperty('yz_cov')) {
        this.yz_cov = initObj.yz_cov
      }
      else {
        this.yz_cov = 0.0;
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
      if (initObj.hasOwnProperty('heading_std_dev')) {
        this.heading_std_dev = initObj.heading_std_dev
      }
      else {
        this.heading_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_std_dev')) {
        this.pitch_std_dev = initObj.pitch_std_dev
      }
      else {
        this.pitch_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('roll_std_dev')) {
        this.roll_std_dev = initObj.roll_std_dev
      }
      else {
        this.roll_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('heading_pitch_cov')) {
        this.heading_pitch_cov = initObj.heading_pitch_cov
      }
      else {
        this.heading_pitch_cov = 0.0;
      }
      if (initObj.hasOwnProperty('heading_roll_cov')) {
        this.heading_roll_cov = initObj.heading_roll_cov
      }
      else {
        this.heading_roll_cov = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_roll_cov')) {
        this.pitch_roll_cov = initObj.pitch_roll_cov
      }
      else {
        this.pitch_roll_cov = 0.0;
      }
      if (initObj.hasOwnProperty('vx')) {
        this.vx = initObj.vx
      }
      else {
        this.vx = 0.0;
      }
      if (initObj.hasOwnProperty('vy')) {
        this.vy = initObj.vy
      }
      else {
        this.vy = 0.0;
      }
      if (initObj.hasOwnProperty('vz')) {
        this.vz = initObj.vz
      }
      else {
        this.vz = 0.0;
      }
      if (initObj.hasOwnProperty('vx_std_dev')) {
        this.vx_std_dev = initObj.vx_std_dev
      }
      else {
        this.vx_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('vy_std_dev')) {
        this.vy_std_dev = initObj.vy_std_dev
      }
      else {
        this.vy_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('vz_std_dev')) {
        this.vz_std_dev = initObj.vz_std_dev
      }
      else {
        this.vz_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('vx_vy_cov')) {
        this.vx_vy_cov = initObj.vx_vy_cov
      }
      else {
        this.vx_vy_cov = 0.0;
      }
      if (initObj.hasOwnProperty('vx_vz_cov')) {
        this.vx_vz_cov = initObj.vx_vz_cov
      }
      else {
        this.vx_vz_cov = 0.0;
      }
      if (initObj.hasOwnProperty('vy_vz_cov')) {
        this.vy_vz_cov = initObj.vy_vz_cov
      }
      else {
        this.vy_vz_cov = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type INSNavCart
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [gnss_mode]
    bufferOffset = _serializer.uint8(obj.gnss_mode, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    // Serialize message field [info]
    bufferOffset = _serializer.uint16(obj.info, buffer, bufferOffset);
    // Serialize message field [gnss_age]
    bufferOffset = _serializer.uint16(obj.gnss_age, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [accuracy]
    bufferOffset = _serializer.uint16(obj.accuracy, buffer, bufferOffset);
    // Serialize message field [latency]
    bufferOffset = _serializer.uint16(obj.latency, buffer, bufferOffset);
    // Serialize message field [datum]
    bufferOffset = _serializer.uint8(obj.datum, buffer, bufferOffset);
    // Serialize message field [sb_list]
    bufferOffset = _serializer.uint16(obj.sb_list, buffer, bufferOffset);
    // Serialize message field [x_std_dev]
    bufferOffset = _serializer.float32(obj.x_std_dev, buffer, bufferOffset);
    // Serialize message field [y_std_dev]
    bufferOffset = _serializer.float32(obj.y_std_dev, buffer, bufferOffset);
    // Serialize message field [z_std_dev]
    bufferOffset = _serializer.float32(obj.z_std_dev, buffer, bufferOffset);
    // Serialize message field [xy_cov]
    bufferOffset = _serializer.float32(obj.xy_cov, buffer, bufferOffset);
    // Serialize message field [xz_cov]
    bufferOffset = _serializer.float32(obj.xz_cov, buffer, bufferOffset);
    // Serialize message field [yz_cov]
    bufferOffset = _serializer.float32(obj.yz_cov, buffer, bufferOffset);
    // Serialize message field [heading]
    bufferOffset = _serializer.float32(obj.heading, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float32(obj.roll, buffer, bufferOffset);
    // Serialize message field [heading_std_dev]
    bufferOffset = _serializer.float32(obj.heading_std_dev, buffer, bufferOffset);
    // Serialize message field [pitch_std_dev]
    bufferOffset = _serializer.float32(obj.pitch_std_dev, buffer, bufferOffset);
    // Serialize message field [roll_std_dev]
    bufferOffset = _serializer.float32(obj.roll_std_dev, buffer, bufferOffset);
    // Serialize message field [heading_pitch_cov]
    bufferOffset = _serializer.float32(obj.heading_pitch_cov, buffer, bufferOffset);
    // Serialize message field [heading_roll_cov]
    bufferOffset = _serializer.float32(obj.heading_roll_cov, buffer, bufferOffset);
    // Serialize message field [pitch_roll_cov]
    bufferOffset = _serializer.float32(obj.pitch_roll_cov, buffer, bufferOffset);
    // Serialize message field [vx]
    bufferOffset = _serializer.float32(obj.vx, buffer, bufferOffset);
    // Serialize message field [vy]
    bufferOffset = _serializer.float32(obj.vy, buffer, bufferOffset);
    // Serialize message field [vz]
    bufferOffset = _serializer.float32(obj.vz, buffer, bufferOffset);
    // Serialize message field [vx_std_dev]
    bufferOffset = _serializer.float32(obj.vx_std_dev, buffer, bufferOffset);
    // Serialize message field [vy_std_dev]
    bufferOffset = _serializer.float32(obj.vy_std_dev, buffer, bufferOffset);
    // Serialize message field [vz_std_dev]
    bufferOffset = _serializer.float32(obj.vz_std_dev, buffer, bufferOffset);
    // Serialize message field [vx_vy_cov]
    bufferOffset = _serializer.float32(obj.vx_vy_cov, buffer, bufferOffset);
    // Serialize message field [vx_vz_cov]
    bufferOffset = _serializer.float32(obj.vx_vz_cov, buffer, bufferOffset);
    // Serialize message field [vy_vz_cov]
    bufferOffset = _serializer.float32(obj.vy_vz_cov, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type INSNavCart
    let len;
    let data = new INSNavCart(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [gnss_mode]
    data.gnss_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [info]
    data.info = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [gnss_age]
    data.gnss_age = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accuracy]
    data.accuracy = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [latency]
    data.latency = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [datum]
    data.datum = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sb_list]
    data.sb_list = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [x_std_dev]
    data.x_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_std_dev]
    data.y_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z_std_dev]
    data.z_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [xy_cov]
    data.xy_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [xz_cov]
    data.xz_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yz_cov]
    data.yz_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading]
    data.heading = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_std_dev]
    data.heading_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_std_dev]
    data.pitch_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_std_dev]
    data.roll_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_pitch_cov]
    data.heading_pitch_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading_roll_cov]
    data.heading_roll_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_roll_cov]
    data.pitch_roll_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vx]
    data.vx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vy]
    data.vy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vz]
    data.vz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vx_std_dev]
    data.vx_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vy_std_dev]
    data.vy_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vz_std_dev]
    data.vz_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vx_vy_cov]
    data.vx_vy_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vx_vz_cov]
    data.vx_vz_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vy_vz_cov]
    data.vy_vz_cov = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 148;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/INSNavCart';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ebb1a1e03ea3fac47ad5f19264160123';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # INSNavCart block 
    # Block_Number 4225
    
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader block_header
    
    uint8       gnss_mode
    uint8       error
    uint16      info
    uint16      gnss_age # 0.01 s
    float64     x        # m
    float64     y        # m
    float64     z        # m
    uint16      accuracy # 0.01 m
    uint16      latency  # 0.1 ms
    uint8       datum    
    #uint8       reserved
    uint16      sb_list
    
    # INSNavCartPosStdDev sub-block definition:
    # If the Position StdDev field is 1 then this sub block is available:
    float32     x_std_dev # m
    float32     y_std_dev # m
    float32     z_std_dev # m
    
    # INSNavCartPosCov sub-block definition:
    # If the Position Cov field is 1 then this sub block is available:
    float32     xy_cov # m^2
    float32     xz_cov # m^2
    float32     yz_cov # m^2
    
    # INSNavCartAtt sub-block definition:
    # If the Attitude field is 1 then this sub block is available:
    float32     heading # deg
    float32     pitch   # deg
    float32     roll    # deg
    
    # INSNavCartAttStdDev sub-block definition:
    # If the Attitude StdDev field is 1 then this sub block is available:
    float32     heading_std_dev # deg
    float32     pitch_std_dev   # deg
    float32     roll_std_dev    # deg
    
    # INSNavCartAttCov sub-block definition:
    # If the Attitude Cov field is 1 then this sub block is available:
    float32     heading_pitch_cov # deg^2
    float32     heading_roll_cov  # deg^2
    float32     pitch_roll_cov    # deg^2
    
    # INSNavCartVel sub-block definition:
    # If the Velocity field is 1 then this sub block is available:
    float32     vx # m/s
    float32     vy # m/S
    float32     vz # m/s
    
    # INSNavCartVelStdDev sub-block definition:
    # If the Velocity StdDev field is 1 then this sub block is available:
    float32     vx_std_dev # m/s
    float32     vy_std_dev # m/s
    float32     vz_std_dev # m/s
    
    # INSNavCartVelCov sub-block definition:
    # If the Velocity Cov field is 1 then this sub block is available:
    float32     vx_vy_cov # m^2/s^2
    float32     vx_vz_cov # m^2/s^2
    float32     vy_vz_cov # m^2/s^2
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
    const resolved = new INSNavCart(null);
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

    if (msg.gnss_mode !== undefined) {
      resolved.gnss_mode = msg.gnss_mode;
    }
    else {
      resolved.gnss_mode = 0
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = 0
    }

    if (msg.info !== undefined) {
      resolved.info = msg.info;
    }
    else {
      resolved.info = 0
    }

    if (msg.gnss_age !== undefined) {
      resolved.gnss_age = msg.gnss_age;
    }
    else {
      resolved.gnss_age = 0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.accuracy !== undefined) {
      resolved.accuracy = msg.accuracy;
    }
    else {
      resolved.accuracy = 0
    }

    if (msg.latency !== undefined) {
      resolved.latency = msg.latency;
    }
    else {
      resolved.latency = 0
    }

    if (msg.datum !== undefined) {
      resolved.datum = msg.datum;
    }
    else {
      resolved.datum = 0
    }

    if (msg.sb_list !== undefined) {
      resolved.sb_list = msg.sb_list;
    }
    else {
      resolved.sb_list = 0
    }

    if (msg.x_std_dev !== undefined) {
      resolved.x_std_dev = msg.x_std_dev;
    }
    else {
      resolved.x_std_dev = 0.0
    }

    if (msg.y_std_dev !== undefined) {
      resolved.y_std_dev = msg.y_std_dev;
    }
    else {
      resolved.y_std_dev = 0.0
    }

    if (msg.z_std_dev !== undefined) {
      resolved.z_std_dev = msg.z_std_dev;
    }
    else {
      resolved.z_std_dev = 0.0
    }

    if (msg.xy_cov !== undefined) {
      resolved.xy_cov = msg.xy_cov;
    }
    else {
      resolved.xy_cov = 0.0
    }

    if (msg.xz_cov !== undefined) {
      resolved.xz_cov = msg.xz_cov;
    }
    else {
      resolved.xz_cov = 0.0
    }

    if (msg.yz_cov !== undefined) {
      resolved.yz_cov = msg.yz_cov;
    }
    else {
      resolved.yz_cov = 0.0
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

    if (msg.heading_std_dev !== undefined) {
      resolved.heading_std_dev = msg.heading_std_dev;
    }
    else {
      resolved.heading_std_dev = 0.0
    }

    if (msg.pitch_std_dev !== undefined) {
      resolved.pitch_std_dev = msg.pitch_std_dev;
    }
    else {
      resolved.pitch_std_dev = 0.0
    }

    if (msg.roll_std_dev !== undefined) {
      resolved.roll_std_dev = msg.roll_std_dev;
    }
    else {
      resolved.roll_std_dev = 0.0
    }

    if (msg.heading_pitch_cov !== undefined) {
      resolved.heading_pitch_cov = msg.heading_pitch_cov;
    }
    else {
      resolved.heading_pitch_cov = 0.0
    }

    if (msg.heading_roll_cov !== undefined) {
      resolved.heading_roll_cov = msg.heading_roll_cov;
    }
    else {
      resolved.heading_roll_cov = 0.0
    }

    if (msg.pitch_roll_cov !== undefined) {
      resolved.pitch_roll_cov = msg.pitch_roll_cov;
    }
    else {
      resolved.pitch_roll_cov = 0.0
    }

    if (msg.vx !== undefined) {
      resolved.vx = msg.vx;
    }
    else {
      resolved.vx = 0.0
    }

    if (msg.vy !== undefined) {
      resolved.vy = msg.vy;
    }
    else {
      resolved.vy = 0.0
    }

    if (msg.vz !== undefined) {
      resolved.vz = msg.vz;
    }
    else {
      resolved.vz = 0.0
    }

    if (msg.vx_std_dev !== undefined) {
      resolved.vx_std_dev = msg.vx_std_dev;
    }
    else {
      resolved.vx_std_dev = 0.0
    }

    if (msg.vy_std_dev !== undefined) {
      resolved.vy_std_dev = msg.vy_std_dev;
    }
    else {
      resolved.vy_std_dev = 0.0
    }

    if (msg.vz_std_dev !== undefined) {
      resolved.vz_std_dev = msg.vz_std_dev;
    }
    else {
      resolved.vz_std_dev = 0.0
    }

    if (msg.vx_vy_cov !== undefined) {
      resolved.vx_vy_cov = msg.vx_vy_cov;
    }
    else {
      resolved.vx_vy_cov = 0.0
    }

    if (msg.vx_vz_cov !== undefined) {
      resolved.vx_vz_cov = msg.vx_vz_cov;
    }
    else {
      resolved.vx_vz_cov = 0.0
    }

    if (msg.vy_vz_cov !== undefined) {
      resolved.vy_vz_cov = msg.vy_vz_cov;
    }
    else {
      resolved.vy_vz_cov = 0.0
    }

    return resolved;
    }
};

module.exports = INSNavCart;
