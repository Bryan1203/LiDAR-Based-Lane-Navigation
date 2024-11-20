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
let VectorInfoCart = require('./VectorInfoCart.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class BaseVectorCart {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_header = null;
      this.n = null;
      this.sb_length = null;
      this.vector_info_cart = null;
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
      if (initObj.hasOwnProperty('vector_info_cart')) {
        this.vector_info_cart = initObj.vector_info_cart
      }
      else {
        this.vector_info_cart = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BaseVectorCart
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_header]
    bufferOffset = BlockHeader.serialize(obj.block_header, buffer, bufferOffset);
    // Serialize message field [n]
    bufferOffset = _serializer.uint8(obj.n, buffer, bufferOffset);
    // Serialize message field [sb_length]
    bufferOffset = _serializer.uint8(obj.sb_length, buffer, bufferOffset);
    // Serialize message field [vector_info_cart]
    // Serialize the length for message field [vector_info_cart]
    bufferOffset = _serializer.uint32(obj.vector_info_cart.length, buffer, bufferOffset);
    obj.vector_info_cart.forEach((val) => {
      bufferOffset = VectorInfoCart.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BaseVectorCart
    let len;
    let data = new BaseVectorCart(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_header]
    data.block_header = BlockHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [n]
    data.n = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sb_length]
    data.sb_length = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vector_info_cart]
    // Deserialize array length for message field [vector_info_cart]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.vector_info_cart = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.vector_info_cart[i] = VectorInfoCart.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 52 * object.vector_info_cart.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/BaseVectorCart';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a80a3938ce87df1c8325f220102dae5d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # BaseVectorCart block 
    # Block_Number 4043
    
    std_msgs/Header header
    
    # SBF block header including time header
    BlockHeader block_header
    
    uint8   n
    uint8   sb_length
    
    VectorInfoCart[] vector_info_cart 
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
    ================================================================================
    MSG: septentrio_gnss_driver/VectorInfoCart
    # VectorInfoCart block
       
    uint8   nr_sv
    uint8   error
    uint8   mode
    uint8   misc
    float64 delta_x      # m
    float64 delta_y      # m
    float64 delta_z      # m
    float32 delta_vx     # m
    float32 delta_vy     # m
    float32 delta_vz     # m
    uint16  azimuth      # 0.01 deg
    int16   elevation    # 0.01 deg
    uint16  reference_id
    uint16  corr_age     # 0.01 s
    uint32  signal_info
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BaseVectorCart(null);
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

    if (msg.vector_info_cart !== undefined) {
      resolved.vector_info_cart = new Array(msg.vector_info_cart.length);
      for (let i = 0; i < resolved.vector_info_cart.length; ++i) {
        resolved.vector_info_cart[i] = VectorInfoCart.Resolve(msg.vector_info_cart[i]);
      }
    }
    else {
      resolved.vector_info_cart = []
    }

    return resolved;
    }
};

module.exports = BaseVectorCart;
