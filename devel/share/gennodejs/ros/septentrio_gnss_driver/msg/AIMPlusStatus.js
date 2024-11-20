// Auto-generated. Do not edit!

// (in-package septentrio_gnss_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AIMPlusStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.tow = null;
      this.wnc = null;
      this.interference = null;
      this.spoofing = null;
      this.osnma_authenticating = null;
      this.galileo_authentic = null;
      this.galileo_spoofed = null;
      this.gps_authentic = null;
      this.gps_spoofed = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('tow')) {
        this.tow = initObj.tow
      }
      else {
        this.tow = 0;
      }
      if (initObj.hasOwnProperty('wnc')) {
        this.wnc = initObj.wnc
      }
      else {
        this.wnc = 0;
      }
      if (initObj.hasOwnProperty('interference')) {
        this.interference = initObj.interference
      }
      else {
        this.interference = 0;
      }
      if (initObj.hasOwnProperty('spoofing')) {
        this.spoofing = initObj.spoofing
      }
      else {
        this.spoofing = 0;
      }
      if (initObj.hasOwnProperty('osnma_authenticating')) {
        this.osnma_authenticating = initObj.osnma_authenticating
      }
      else {
        this.osnma_authenticating = false;
      }
      if (initObj.hasOwnProperty('galileo_authentic')) {
        this.galileo_authentic = initObj.galileo_authentic
      }
      else {
        this.galileo_authentic = 0;
      }
      if (initObj.hasOwnProperty('galileo_spoofed')) {
        this.galileo_spoofed = initObj.galileo_spoofed
      }
      else {
        this.galileo_spoofed = 0;
      }
      if (initObj.hasOwnProperty('gps_authentic')) {
        this.gps_authentic = initObj.gps_authentic
      }
      else {
        this.gps_authentic = 0;
      }
      if (initObj.hasOwnProperty('gps_spoofed')) {
        this.gps_spoofed = initObj.gps_spoofed
      }
      else {
        this.gps_spoofed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AIMPlusStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [tow]
    bufferOffset = _serializer.uint32(obj.tow, buffer, bufferOffset);
    // Serialize message field [wnc]
    bufferOffset = _serializer.uint16(obj.wnc, buffer, bufferOffset);
    // Serialize message field [interference]
    bufferOffset = _serializer.uint8(obj.interference, buffer, bufferOffset);
    // Serialize message field [spoofing]
    bufferOffset = _serializer.uint8(obj.spoofing, buffer, bufferOffset);
    // Serialize message field [osnma_authenticating]
    bufferOffset = _serializer.bool(obj.osnma_authenticating, buffer, bufferOffset);
    // Serialize message field [galileo_authentic]
    bufferOffset = _serializer.uint8(obj.galileo_authentic, buffer, bufferOffset);
    // Serialize message field [galileo_spoofed]
    bufferOffset = _serializer.uint8(obj.galileo_spoofed, buffer, bufferOffset);
    // Serialize message field [gps_authentic]
    bufferOffset = _serializer.uint8(obj.gps_authentic, buffer, bufferOffset);
    // Serialize message field [gps_spoofed]
    bufferOffset = _serializer.uint8(obj.gps_spoofed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AIMPlusStatus
    let len;
    let data = new AIMPlusStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [tow]
    data.tow = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [wnc]
    data.wnc = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [interference]
    data.interference = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [spoofing]
    data.spoofing = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [osnma_authenticating]
    data.osnma_authenticating = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [galileo_authentic]
    data.galileo_authentic = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [galileo_spoofed]
    data.galileo_spoofed = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gps_authentic]
    data.gps_authentic = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gps_spoofed]
    data.gps_spoofed = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/AIMPlusStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '85cf4fc2fc2598dca0b9bb856cb2dcdd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # AIMPlusStatus message
    # ROS message header
    std_msgs/Header header
    
    uint32 tow # ms since week start
    uint16 wnc # weeks since Jan 06, 1980 at 00:00:00 UTC     
    
    uint8 interference
    #-------------------------------
    uint8 SPECTRUM_CLEAN         = 0
    uint8 INTERFERENCE_MITIGATED = 1
    uint8 INTERFERENCE_PRESENT   = 2
    #-------------------------------
    
    uint8 spoofing
    #--------------------------------------------------------
    uint8 NONE_DETECTED                                   = 0
    uint8 SPOOFING_DETECTED_BY_OSNMA                      = 1
    uint8 SPOOFING_DETECTED_BY_AUTHENTCITY_TEST           = 2
    uint8 SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST = 3
    #--------------------------------------------------------
    
    bool  osnma_authenticating
    uint8 galileo_authentic
    uint8 galileo_spoofed
    uint8 gps_authentic
    uint8 gps_spoofed
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AIMPlusStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.tow !== undefined) {
      resolved.tow = msg.tow;
    }
    else {
      resolved.tow = 0
    }

    if (msg.wnc !== undefined) {
      resolved.wnc = msg.wnc;
    }
    else {
      resolved.wnc = 0
    }

    if (msg.interference !== undefined) {
      resolved.interference = msg.interference;
    }
    else {
      resolved.interference = 0
    }

    if (msg.spoofing !== undefined) {
      resolved.spoofing = msg.spoofing;
    }
    else {
      resolved.spoofing = 0
    }

    if (msg.osnma_authenticating !== undefined) {
      resolved.osnma_authenticating = msg.osnma_authenticating;
    }
    else {
      resolved.osnma_authenticating = false
    }

    if (msg.galileo_authentic !== undefined) {
      resolved.galileo_authentic = msg.galileo_authentic;
    }
    else {
      resolved.galileo_authentic = 0
    }

    if (msg.galileo_spoofed !== undefined) {
      resolved.galileo_spoofed = msg.galileo_spoofed;
    }
    else {
      resolved.galileo_spoofed = 0
    }

    if (msg.gps_authentic !== undefined) {
      resolved.gps_authentic = msg.gps_authentic;
    }
    else {
      resolved.gps_authentic = 0
    }

    if (msg.gps_spoofed !== undefined) {
      resolved.gps_spoofed = msg.gps_spoofed;
    }
    else {
      resolved.gps_spoofed = 0
    }

    return resolved;
    }
};

// Constants for message
AIMPlusStatus.Constants = {
  SPECTRUM_CLEAN: 0,
  INTERFERENCE_MITIGATED: 1,
  INTERFERENCE_PRESENT: 2,
  NONE_DETECTED: 0,
  SPOOFING_DETECTED_BY_OSNMA: 1,
  SPOOFING_DETECTED_BY_AUTHENTCITY_TEST: 2,
  SPOOFING_DETECTED_BY_OSNMA_AND_AUTHENTCITY_TEST: 3,
}

module.exports = AIMPlusStatus;
