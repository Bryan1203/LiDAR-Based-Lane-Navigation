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

class PVTCartesian {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.mode = null;
      this.error = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.undulation = null;
      this.vx = null;
      this.vy = null;
      this.vz = null;
      this.cog = null;
      this.rx_clk_bias = null;
      this.rx_clk_drift = null;
      this.time_system = null;
      this.datum = null;
      this.nr_sv = null;
      this.wa_corr_info = null;
      this.reference_id = null;
      this.mean_corr_age = null;
      this.signal_info = null;
      this.alert_flag = null;
      this.nr_bases = null;
      this.ppp_info = null;
      this.latency = null;
      this.h_accuracy = null;
      this.v_accuracy = null;
      this.misc = null;
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
      if (initObj.hasOwnProperty('undulation')) {
        this.undulation = initObj.undulation
      }
      else {
        this.undulation = 0.0;
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
      if (initObj.hasOwnProperty('cog')) {
        this.cog = initObj.cog
      }
      else {
        this.cog = 0.0;
      }
      if (initObj.hasOwnProperty('rx_clk_bias')) {
        this.rx_clk_bias = initObj.rx_clk_bias
      }
      else {
        this.rx_clk_bias = 0.0;
      }
      if (initObj.hasOwnProperty('rx_clk_drift')) {
        this.rx_clk_drift = initObj.rx_clk_drift
      }
      else {
        this.rx_clk_drift = 0.0;
      }
      if (initObj.hasOwnProperty('time_system')) {
        this.time_system = initObj.time_system
      }
      else {
        this.time_system = 0;
      }
      if (initObj.hasOwnProperty('datum')) {
        this.datum = initObj.datum
      }
      else {
        this.datum = 0;
      }
      if (initObj.hasOwnProperty('nr_sv')) {
        this.nr_sv = initObj.nr_sv
      }
      else {
        this.nr_sv = 0;
      }
      if (initObj.hasOwnProperty('wa_corr_info')) {
        this.wa_corr_info = initObj.wa_corr_info
      }
      else {
        this.wa_corr_info = 0;
      }
      if (initObj.hasOwnProperty('reference_id')) {
        this.reference_id = initObj.reference_id
      }
      else {
        this.reference_id = 0;
      }
      if (initObj.hasOwnProperty('mean_corr_age')) {
        this.mean_corr_age = initObj.mean_corr_age
      }
      else {
        this.mean_corr_age = 0;
      }
      if (initObj.hasOwnProperty('signal_info')) {
        this.signal_info = initObj.signal_info
      }
      else {
        this.signal_info = 0;
      }
      if (initObj.hasOwnProperty('alert_flag')) {
        this.alert_flag = initObj.alert_flag
      }
      else {
        this.alert_flag = 0;
      }
      if (initObj.hasOwnProperty('nr_bases')) {
        this.nr_bases = initObj.nr_bases
      }
      else {
        this.nr_bases = 0;
      }
      if (initObj.hasOwnProperty('ppp_info')) {
        this.ppp_info = initObj.ppp_info
      }
      else {
        this.ppp_info = 0;
      }
      if (initObj.hasOwnProperty('latency')) {
        this.latency = initObj.latency
      }
      else {
        this.latency = 0;
      }
      if (initObj.hasOwnProperty('h_accuracy')) {
        this.h_accuracy = initObj.h_accuracy
      }
      else {
        this.h_accuracy = 0;
      }
      if (initObj.hasOwnProperty('v_accuracy')) {
        this.v_accuracy = initObj.v_accuracy
      }
      else {
        this.v_accuracy = 0;
      }
      if (initObj.hasOwnProperty('misc')) {
        this.misc = initObj.misc
      }
      else {
        this.misc = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PVTCartesian
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [undulation]
    bufferOffset = _serializer.float32(obj.undulation, buffer, bufferOffset);
    // Serialize message field [vx]
    bufferOffset = _serializer.float32(obj.vx, buffer, bufferOffset);
    // Serialize message field [vy]
    bufferOffset = _serializer.float32(obj.vy, buffer, bufferOffset);
    // Serialize message field [vz]
    bufferOffset = _serializer.float32(obj.vz, buffer, bufferOffset);
    // Serialize message field [cog]
    bufferOffset = _serializer.float32(obj.cog, buffer, bufferOffset);
    // Serialize message field [rx_clk_bias]
    bufferOffset = _serializer.float64(obj.rx_clk_bias, buffer, bufferOffset);
    // Serialize message field [rx_clk_drift]
    bufferOffset = _serializer.float32(obj.rx_clk_drift, buffer, bufferOffset);
    // Serialize message field [time_system]
    bufferOffset = _serializer.uint8(obj.time_system, buffer, bufferOffset);
    // Serialize message field [datum]
    bufferOffset = _serializer.uint8(obj.datum, buffer, bufferOffset);
    // Serialize message field [nr_sv]
    bufferOffset = _serializer.uint8(obj.nr_sv, buffer, bufferOffset);
    // Serialize message field [wa_corr_info]
    bufferOffset = _serializer.uint8(obj.wa_corr_info, buffer, bufferOffset);
    // Serialize message field [reference_id]
    bufferOffset = _serializer.uint16(obj.reference_id, buffer, bufferOffset);
    // Serialize message field [mean_corr_age]
    bufferOffset = _serializer.uint16(obj.mean_corr_age, buffer, bufferOffset);
    // Serialize message field [signal_info]
    bufferOffset = _serializer.uint32(obj.signal_info, buffer, bufferOffset);
    // Serialize message field [alert_flag]
    bufferOffset = _serializer.uint8(obj.alert_flag, buffer, bufferOffset);
    // Serialize message field [nr_bases]
    bufferOffset = _serializer.uint8(obj.nr_bases, buffer, bufferOffset);
    // Serialize message field [ppp_info]
    bufferOffset = _serializer.uint16(obj.ppp_info, buffer, bufferOffset);
    // Serialize message field [latency]
    bufferOffset = _serializer.uint16(obj.latency, buffer, bufferOffset);
    // Serialize message field [h_accuracy]
    bufferOffset = _serializer.uint16(obj.h_accuracy, buffer, bufferOffset);
    // Serialize message field [v_accuracy]
    bufferOffset = _serializer.uint16(obj.v_accuracy, buffer, bufferOffset);
    // Serialize message field [misc]
    bufferOffset = _serializer.uint8(obj.misc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PVTCartesian
    let len;
    let data = new PVTCartesian(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [undulation]
    data.undulation = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vx]
    data.vx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vy]
    data.vy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vz]
    data.vz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [cog]
    data.cog = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rx_clk_bias]
    data.rx_clk_bias = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rx_clk_drift]
    data.rx_clk_drift = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [time_system]
    data.time_system = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [datum]
    data.datum = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [nr_sv]
    data.nr_sv = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [wa_corr_info]
    data.wa_corr_info = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [reference_id]
    data.reference_id = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [mean_corr_age]
    data.mean_corr_age = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [signal_info]
    data.signal_info = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [alert_flag]
    data.alert_flag = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [nr_bases]
    data.nr_bases = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [ppp_info]
    data.ppp_info = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [latency]
    data.latency = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [h_accuracy]
    data.h_accuracy = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [v_accuracy]
    data.v_accuracy = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [misc]
    data.misc = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/PVTCartesian';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '98b7f07c88704d34a9797ee6019c2d54';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # PVTCartesian block
    # ROS message header
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader  block_header   
    
    uint8        mode
    uint8        error        
    float64      x             # m    
    float64      y             # m             
    float64      z             # m      
    float32      undulation    # m 
    float32      vx            # m/s         
    float32      vy            # m/s 
    float32      vz            # m/s
    float32      cog           # deg
    float64      rx_clk_bias   # ms
    float32      rx_clk_drift  # ppm
    uint8        time_system 
    uint8        datum
    uint8        nr_sv 
    uint8        wa_corr_info
    uint16       reference_id
    uint16       mean_corr_age # 0.01s
    uint32       signal_info
    uint8        alert_flag
    uint8        nr_bases
    uint16       ppp_info      # s
    uint16       latency       # 0.0001 s
    uint16       h_accuracy    # 0.01 m
    uint16       v_accuracy    # 0.01 m
    uint8        misc
    
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
    const resolved = new PVTCartesian(null);
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

    if (msg.undulation !== undefined) {
      resolved.undulation = msg.undulation;
    }
    else {
      resolved.undulation = 0.0
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

    if (msg.cog !== undefined) {
      resolved.cog = msg.cog;
    }
    else {
      resolved.cog = 0.0
    }

    if (msg.rx_clk_bias !== undefined) {
      resolved.rx_clk_bias = msg.rx_clk_bias;
    }
    else {
      resolved.rx_clk_bias = 0.0
    }

    if (msg.rx_clk_drift !== undefined) {
      resolved.rx_clk_drift = msg.rx_clk_drift;
    }
    else {
      resolved.rx_clk_drift = 0.0
    }

    if (msg.time_system !== undefined) {
      resolved.time_system = msg.time_system;
    }
    else {
      resolved.time_system = 0
    }

    if (msg.datum !== undefined) {
      resolved.datum = msg.datum;
    }
    else {
      resolved.datum = 0
    }

    if (msg.nr_sv !== undefined) {
      resolved.nr_sv = msg.nr_sv;
    }
    else {
      resolved.nr_sv = 0
    }

    if (msg.wa_corr_info !== undefined) {
      resolved.wa_corr_info = msg.wa_corr_info;
    }
    else {
      resolved.wa_corr_info = 0
    }

    if (msg.reference_id !== undefined) {
      resolved.reference_id = msg.reference_id;
    }
    else {
      resolved.reference_id = 0
    }

    if (msg.mean_corr_age !== undefined) {
      resolved.mean_corr_age = msg.mean_corr_age;
    }
    else {
      resolved.mean_corr_age = 0
    }

    if (msg.signal_info !== undefined) {
      resolved.signal_info = msg.signal_info;
    }
    else {
      resolved.signal_info = 0
    }

    if (msg.alert_flag !== undefined) {
      resolved.alert_flag = msg.alert_flag;
    }
    else {
      resolved.alert_flag = 0
    }

    if (msg.nr_bases !== undefined) {
      resolved.nr_bases = msg.nr_bases;
    }
    else {
      resolved.nr_bases = 0
    }

    if (msg.ppp_info !== undefined) {
      resolved.ppp_info = msg.ppp_info;
    }
    else {
      resolved.ppp_info = 0
    }

    if (msg.latency !== undefined) {
      resolved.latency = msg.latency;
    }
    else {
      resolved.latency = 0
    }

    if (msg.h_accuracy !== undefined) {
      resolved.h_accuracy = msg.h_accuracy;
    }
    else {
      resolved.h_accuracy = 0
    }

    if (msg.v_accuracy !== undefined) {
      resolved.v_accuracy = msg.v_accuracy;
    }
    else {
      resolved.v_accuracy = 0
    }

    if (msg.misc !== undefined) {
      resolved.misc = msg.misc;
    }
    else {
      resolved.misc = 0
    }

    return resolved;
    }
};

module.exports = PVTCartesian;
