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

class BlockHeader {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sync_1 = null;
      this.sync_2 = null;
      this.crc = null;
      this.id = null;
      this.revision = null;
      this.length = null;
      this.tow = null;
      this.wnc = null;
    }
    else {
      if (initObj.hasOwnProperty('sync_1')) {
        this.sync_1 = initObj.sync_1
      }
      else {
        this.sync_1 = 0;
      }
      if (initObj.hasOwnProperty('sync_2')) {
        this.sync_2 = initObj.sync_2
      }
      else {
        this.sync_2 = 0;
      }
      if (initObj.hasOwnProperty('crc')) {
        this.crc = initObj.crc
      }
      else {
        this.crc = 0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('revision')) {
        this.revision = initObj.revision
      }
      else {
        this.revision = 0;
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BlockHeader
    // Serialize message field [sync_1]
    bufferOffset = _serializer.uint8(obj.sync_1, buffer, bufferOffset);
    // Serialize message field [sync_2]
    bufferOffset = _serializer.uint8(obj.sync_2, buffer, bufferOffset);
    // Serialize message field [crc]
    bufferOffset = _serializer.uint16(obj.crc, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint16(obj.id, buffer, bufferOffset);
    // Serialize message field [revision]
    bufferOffset = _serializer.uint8(obj.revision, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.uint16(obj.length, buffer, bufferOffset);
    // Serialize message field [tow]
    bufferOffset = _serializer.uint32(obj.tow, buffer, bufferOffset);
    // Serialize message field [wnc]
    bufferOffset = _serializer.uint16(obj.wnc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BlockHeader
    let len;
    let data = new BlockHeader(null);
    // Deserialize message field [sync_1]
    data.sync_1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sync_2]
    data.sync_2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [crc]
    data.crc = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [revision]
    data.revision = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [tow]
    data.tow = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [wnc]
    data.wnc = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/BlockHeader';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '518ce7a1f9cbde747cc0c5b017e06f70';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new BlockHeader(null);
    if (msg.sync_1 !== undefined) {
      resolved.sync_1 = msg.sync_1;
    }
    else {
      resolved.sync_1 = 0
    }

    if (msg.sync_2 !== undefined) {
      resolved.sync_2 = msg.sync_2;
    }
    else {
      resolved.sync_2 = 0
    }

    if (msg.crc !== undefined) {
      resolved.crc = msg.crc;
    }
    else {
      resolved.crc = 0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.revision !== undefined) {
      resolved.revision = msg.revision;
    }
    else {
      resolved.revision = 0
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0
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

    return resolved;
    }
};

module.exports = BlockHeader;
