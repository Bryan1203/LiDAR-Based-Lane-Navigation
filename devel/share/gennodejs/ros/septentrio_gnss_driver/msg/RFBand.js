// Auto-generated. Do not edit!

// (in-package septentrio_gnss_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class RFBand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frequency = null;
      this.bandwidth = null;
      this.info = null;
    }
    else {
      if (initObj.hasOwnProperty('frequency')) {
        this.frequency = initObj.frequency
      }
      else {
        this.frequency = 0;
      }
      if (initObj.hasOwnProperty('bandwidth')) {
        this.bandwidth = initObj.bandwidth
      }
      else {
        this.bandwidth = 0;
      }
      if (initObj.hasOwnProperty('info')) {
        this.info = initObj.info
      }
      else {
        this.info = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RFBand
    // Serialize message field [frequency]
    bufferOffset = _serializer.uint32(obj.frequency, buffer, bufferOffset);
    // Serialize message field [bandwidth]
    bufferOffset = _serializer.uint16(obj.bandwidth, buffer, bufferOffset);
    // Serialize message field [info]
    bufferOffset = _serializer.uint8(obj.info, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RFBand
    let len;
    let data = new RFBand(null);
    // Deserialize message field [frequency]
    data.frequency = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [bandwidth]
    data.bandwidth = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [info]
    data.info = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/RFBand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '987cd35fc563b11daae475e3e9a37fd6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # RFband block
    
    uint32 frequency # Hz
    uint16 bandwidth # kHz
    uint8  info
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RFBand(null);
    if (msg.frequency !== undefined) {
      resolved.frequency = msg.frequency;
    }
    else {
      resolved.frequency = 0
    }

    if (msg.bandwidth !== undefined) {
      resolved.bandwidth = msg.bandwidth;
    }
    else {
      resolved.bandwidth = 0
    }

    if (msg.info !== undefined) {
      resolved.info = msg.info;
    }
    else {
      resolved.info = 0
    }

    return resolved;
    }
};

module.exports = RFBand;
