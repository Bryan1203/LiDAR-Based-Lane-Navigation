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

class INSNavGeod {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.gnss_mode = null;
      this.error = null;
      this.info = null;
      this.gnss_age = null;
      this.latitude = null;
      this.longitude = null;
      this.height = null;
      this.undulation = null;
      this.accuracy = null;
      this.latency = null;
      this.datum = null;
      this.sb_list = null;
      this.latitude_std_dev = null;
      this.longitude_std_dev = null;
      this.height_std_dev = null;
      this.latitude_longitude_cov = null;
      this.latitude_height_cov = null;
      this.longitude_height_cov = null;
      this.heading = null;
      this.pitch = null;
      this.roll = null;
      this.heading_std_dev = null;
      this.pitch_std_dev = null;
      this.roll_std_dev = null;
      this.heading_pitch_cov = null;
      this.heading_roll_cov = null;
      this.pitch_roll_cov = null;
      this.ve = null;
      this.vn = null;
      this.vu = null;
      this.ve_std_dev = null;
      this.vn_std_dev = null;
      this.vu_std_dev = null;
      this.ve_vn_cov = null;
      this.ve_vu_cov = null;
      this.vn_vu_cov = null;
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
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0.0;
      }
      if (initObj.hasOwnProperty('undulation')) {
        this.undulation = initObj.undulation
      }
      else {
        this.undulation = 0.0;
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
      if (initObj.hasOwnProperty('latitude_std_dev')) {
        this.latitude_std_dev = initObj.latitude_std_dev
      }
      else {
        this.latitude_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('longitude_std_dev')) {
        this.longitude_std_dev = initObj.longitude_std_dev
      }
      else {
        this.longitude_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('height_std_dev')) {
        this.height_std_dev = initObj.height_std_dev
      }
      else {
        this.height_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('latitude_longitude_cov')) {
        this.latitude_longitude_cov = initObj.latitude_longitude_cov
      }
      else {
        this.latitude_longitude_cov = 0.0;
      }
      if (initObj.hasOwnProperty('latitude_height_cov')) {
        this.latitude_height_cov = initObj.latitude_height_cov
      }
      else {
        this.latitude_height_cov = 0.0;
      }
      if (initObj.hasOwnProperty('longitude_height_cov')) {
        this.longitude_height_cov = initObj.longitude_height_cov
      }
      else {
        this.longitude_height_cov = 0.0;
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
      if (initObj.hasOwnProperty('ve')) {
        this.ve = initObj.ve
      }
      else {
        this.ve = 0.0;
      }
      if (initObj.hasOwnProperty('vn')) {
        this.vn = initObj.vn
      }
      else {
        this.vn = 0.0;
      }
      if (initObj.hasOwnProperty('vu')) {
        this.vu = initObj.vu
      }
      else {
        this.vu = 0.0;
      }
      if (initObj.hasOwnProperty('ve_std_dev')) {
        this.ve_std_dev = initObj.ve_std_dev
      }
      else {
        this.ve_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('vn_std_dev')) {
        this.vn_std_dev = initObj.vn_std_dev
      }
      else {
        this.vn_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('vu_std_dev')) {
        this.vu_std_dev = initObj.vu_std_dev
      }
      else {
        this.vu_std_dev = 0.0;
      }
      if (initObj.hasOwnProperty('ve_vn_cov')) {
        this.ve_vn_cov = initObj.ve_vn_cov
      }
      else {
        this.ve_vn_cov = 0.0;
      }
      if (initObj.hasOwnProperty('ve_vu_cov')) {
        this.ve_vu_cov = initObj.ve_vu_cov
      }
      else {
        this.ve_vu_cov = 0.0;
      }
      if (initObj.hasOwnProperty('vn_vu_cov')) {
        this.vn_vu_cov = initObj.vn_vu_cov
      }
      else {
        this.vn_vu_cov = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type INSNavGeod
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
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.float64(obj.height, buffer, bufferOffset);
    // Serialize message field [undulation]
    bufferOffset = _serializer.float32(obj.undulation, buffer, bufferOffset);
    // Serialize message field [accuracy]
    bufferOffset = _serializer.uint16(obj.accuracy, buffer, bufferOffset);
    // Serialize message field [latency]
    bufferOffset = _serializer.uint16(obj.latency, buffer, bufferOffset);
    // Serialize message field [datum]
    bufferOffset = _serializer.uint8(obj.datum, buffer, bufferOffset);
    // Serialize message field [sb_list]
    bufferOffset = _serializer.uint16(obj.sb_list, buffer, bufferOffset);
    // Serialize message field [latitude_std_dev]
    bufferOffset = _serializer.float32(obj.latitude_std_dev, buffer, bufferOffset);
    // Serialize message field [longitude_std_dev]
    bufferOffset = _serializer.float32(obj.longitude_std_dev, buffer, bufferOffset);
    // Serialize message field [height_std_dev]
    bufferOffset = _serializer.float32(obj.height_std_dev, buffer, bufferOffset);
    // Serialize message field [latitude_longitude_cov]
    bufferOffset = _serializer.float32(obj.latitude_longitude_cov, buffer, bufferOffset);
    // Serialize message field [latitude_height_cov]
    bufferOffset = _serializer.float32(obj.latitude_height_cov, buffer, bufferOffset);
    // Serialize message field [longitude_height_cov]
    bufferOffset = _serializer.float32(obj.longitude_height_cov, buffer, bufferOffset);
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
    // Serialize message field [ve]
    bufferOffset = _serializer.float32(obj.ve, buffer, bufferOffset);
    // Serialize message field [vn]
    bufferOffset = _serializer.float32(obj.vn, buffer, bufferOffset);
    // Serialize message field [vu]
    bufferOffset = _serializer.float32(obj.vu, buffer, bufferOffset);
    // Serialize message field [ve_std_dev]
    bufferOffset = _serializer.float32(obj.ve_std_dev, buffer, bufferOffset);
    // Serialize message field [vn_std_dev]
    bufferOffset = _serializer.float32(obj.vn_std_dev, buffer, bufferOffset);
    // Serialize message field [vu_std_dev]
    bufferOffset = _serializer.float32(obj.vu_std_dev, buffer, bufferOffset);
    // Serialize message field [ve_vn_cov]
    bufferOffset = _serializer.float32(obj.ve_vn_cov, buffer, bufferOffset);
    // Serialize message field [ve_vu_cov]
    bufferOffset = _serializer.float32(obj.ve_vu_cov, buffer, bufferOffset);
    // Serialize message field [vn_vu_cov]
    bufferOffset = _serializer.float32(obj.vn_vu_cov, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type INSNavGeod
    let len;
    let data = new INSNavGeod(null);
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
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [undulation]
    data.undulation = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [accuracy]
    data.accuracy = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [latency]
    data.latency = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [datum]
    data.datum = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sb_list]
    data.sb_list = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [latitude_std_dev]
    data.latitude_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [longitude_std_dev]
    data.longitude_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [height_std_dev]
    data.height_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [latitude_longitude_cov]
    data.latitude_longitude_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [latitude_height_cov]
    data.latitude_height_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [longitude_height_cov]
    data.longitude_height_cov = _deserializer.float32(buffer, bufferOffset);
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
    // Deserialize message field [ve]
    data.ve = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vn]
    data.vn = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vu]
    data.vu = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ve_std_dev]
    data.ve_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vn_std_dev]
    data.vn_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vu_std_dev]
    data.vu_std_dev = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ve_vn_cov]
    data.ve_vn_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ve_vu_cov]
    data.ve_vu_cov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vn_vu_cov]
    data.vn_vu_cov = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 152;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/INSNavGeod';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e95cd9c24ef355a547323f4ca3d6e14';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # INSNavGeod block 
    # Block_Number 4226
    
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader block_header
    
    uint8       gnss_mode
    uint8       error
    uint16      info
    uint16      gnss_age   # 0.01 s
    float64     latitude   # rad
    float64     longitude  # rad
    float64     height     # m (ellipsoidal)
    float32     undulation # m
    uint16      accuracy   # 0.01 m
    uint16      latency    # 0.0001 s
    uint8       datum
    #uint8       reserved
    uint16      sb_list
    
    # INSNavGeodPosStdDev sub-block definition:
    # If the Position StdDev field is 1 then this sub block is available:
    float32     latitude_std_dev  # m
    float32     longitude_std_dev # m
    float32     height_std_dev    # m
    
    # INSNavGeodPosCov sub-block definition:
    # If the Position Cov field is 1 then this sub block is available:
    float32     latitude_longitude_cov # m^2
    float32     latitude_height_cov    # m^2
    float32     longitude_height_cov   # m^2
    
    # INSNavGeodAtt sub-block definition:
    # If the Attitude field is 1 then this sub block is available:
    float32     heading # deg
    float32     pitch   # deg
    float32     roll    # deg
    
    # INSNavGeodAttStdDev sub-block definition:
    # If the Attitude StdDev field is 1 then this sub block is available:
    float32     heading_std_dev # deg
    float32     pitch_std_dev   # deg
    float32     roll_std_dev    # deg
    
    # INSNavGeodAttCov sub-block definition:
    # If the Attitude Cov field is 1 then this sub block is available:
    float32     heading_pitch_cov # deg^2
    float32     heading_roll_cov  # deg^2
    float32     pitch_roll_cov    # deg^2
    
    # INSNavGeodVel sub-block definition:
    # If the Velocity field is 1 then this sub block is available:
    float32     ve # m/s
    float32     vn # m/s 
    float32     vu # m/s
    
    # INSNavGeodVelStdDev sub-block definition:
    # If the Velocity StdDev field is 1 then this sub block is available:
    float32     ve_std_dev # m/s
    float32     vn_std_dev # m/s
    float32     vu_std_dev # m/s
    
    # INSNavGeodVelCov sub-block definition:
    # If the Velocity Cov field is 1 then this sub block is available:
    float32     ve_vn_cov # m^2/s^2
    float32     ve_vu_cov # m^2/s^2
    float32     vn_vu_cov # m^2/s^2   
    
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
    const resolved = new INSNavGeod(null);
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

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0.0
    }

    if (msg.undulation !== undefined) {
      resolved.undulation = msg.undulation;
    }
    else {
      resolved.undulation = 0.0
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

    if (msg.latitude_std_dev !== undefined) {
      resolved.latitude_std_dev = msg.latitude_std_dev;
    }
    else {
      resolved.latitude_std_dev = 0.0
    }

    if (msg.longitude_std_dev !== undefined) {
      resolved.longitude_std_dev = msg.longitude_std_dev;
    }
    else {
      resolved.longitude_std_dev = 0.0
    }

    if (msg.height_std_dev !== undefined) {
      resolved.height_std_dev = msg.height_std_dev;
    }
    else {
      resolved.height_std_dev = 0.0
    }

    if (msg.latitude_longitude_cov !== undefined) {
      resolved.latitude_longitude_cov = msg.latitude_longitude_cov;
    }
    else {
      resolved.latitude_longitude_cov = 0.0
    }

    if (msg.latitude_height_cov !== undefined) {
      resolved.latitude_height_cov = msg.latitude_height_cov;
    }
    else {
      resolved.latitude_height_cov = 0.0
    }

    if (msg.longitude_height_cov !== undefined) {
      resolved.longitude_height_cov = msg.longitude_height_cov;
    }
    else {
      resolved.longitude_height_cov = 0.0
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

    if (msg.ve !== undefined) {
      resolved.ve = msg.ve;
    }
    else {
      resolved.ve = 0.0
    }

    if (msg.vn !== undefined) {
      resolved.vn = msg.vn;
    }
    else {
      resolved.vn = 0.0
    }

    if (msg.vu !== undefined) {
      resolved.vu = msg.vu;
    }
    else {
      resolved.vu = 0.0
    }

    if (msg.ve_std_dev !== undefined) {
      resolved.ve_std_dev = msg.ve_std_dev;
    }
    else {
      resolved.ve_std_dev = 0.0
    }

    if (msg.vn_std_dev !== undefined) {
      resolved.vn_std_dev = msg.vn_std_dev;
    }
    else {
      resolved.vn_std_dev = 0.0
    }

    if (msg.vu_std_dev !== undefined) {
      resolved.vu_std_dev = msg.vu_std_dev;
    }
    else {
      resolved.vu_std_dev = 0.0
    }

    if (msg.ve_vn_cov !== undefined) {
      resolved.ve_vn_cov = msg.ve_vn_cov;
    }
    else {
      resolved.ve_vn_cov = 0.0
    }

    if (msg.ve_vu_cov !== undefined) {
      resolved.ve_vu_cov = msg.ve_vu_cov;
    }
    else {
      resolved.ve_vu_cov = 0.0
    }

    if (msg.vn_vu_cov !== undefined) {
      resolved.vn_vu_cov = msg.vn_vu_cov;
    }
    else {
      resolved.vn_vu_cov = 0.0
    }

    return resolved;
    }
};

module.exports = INSNavGeod;
