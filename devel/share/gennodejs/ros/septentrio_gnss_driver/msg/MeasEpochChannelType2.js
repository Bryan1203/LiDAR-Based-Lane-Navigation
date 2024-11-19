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

class MeasEpochChannelType2 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.lock_time = null;
      this.cn0 = null;
      this.offsets_msb = null;
      this.carrier_msb = null;
      this.obs_info = null;
      this.code_offset_lsb = null;
      this.carrier_lsb = null;
      this.doppler_offset_lsb = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('lock_time')) {
        this.lock_time = initObj.lock_time
      }
      else {
        this.lock_time = 0;
      }
      if (initObj.hasOwnProperty('cn0')) {
        this.cn0 = initObj.cn0
      }
      else {
        this.cn0 = 0;
      }
      if (initObj.hasOwnProperty('offsets_msb')) {
        this.offsets_msb = initObj.offsets_msb
      }
      else {
        this.offsets_msb = 0;
      }
      if (initObj.hasOwnProperty('carrier_msb')) {
        this.carrier_msb = initObj.carrier_msb
      }
      else {
        this.carrier_msb = 0;
      }
      if (initObj.hasOwnProperty('obs_info')) {
        this.obs_info = initObj.obs_info
      }
      else {
        this.obs_info = 0;
      }
      if (initObj.hasOwnProperty('code_offset_lsb')) {
        this.code_offset_lsb = initObj.code_offset_lsb
      }
      else {
        this.code_offset_lsb = 0;
      }
      if (initObj.hasOwnProperty('carrier_lsb')) {
        this.carrier_lsb = initObj.carrier_lsb
      }
      else {
        this.carrier_lsb = 0;
      }
      if (initObj.hasOwnProperty('doppler_offset_lsb')) {
        this.doppler_offset_lsb = initObj.doppler_offset_lsb
      }
      else {
        this.doppler_offset_lsb = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MeasEpochChannelType2
    // Serialize message field [type]
    bufferOffset = _serializer.uint8(obj.type, buffer, bufferOffset);
    // Serialize message field [lock_time]
    bufferOffset = _serializer.uint8(obj.lock_time, buffer, bufferOffset);
    // Serialize message field [cn0]
    bufferOffset = _serializer.uint8(obj.cn0, buffer, bufferOffset);
    // Serialize message field [offsets_msb]
    bufferOffset = _serializer.uint8(obj.offsets_msb, buffer, bufferOffset);
    // Serialize message field [carrier_msb]
    bufferOffset = _serializer.int8(obj.carrier_msb, buffer, bufferOffset);
    // Serialize message field [obs_info]
    bufferOffset = _serializer.uint8(obj.obs_info, buffer, bufferOffset);
    // Serialize message field [code_offset_lsb]
    bufferOffset = _serializer.uint16(obj.code_offset_lsb, buffer, bufferOffset);
    // Serialize message field [carrier_lsb]
    bufferOffset = _serializer.uint16(obj.carrier_lsb, buffer, bufferOffset);
    // Serialize message field [doppler_offset_lsb]
    bufferOffset = _serializer.uint16(obj.doppler_offset_lsb, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MeasEpochChannelType2
    let len;
    let data = new MeasEpochChannelType2(null);
    // Deserialize message field [type]
    data.type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lock_time]
    data.lock_time = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cn0]
    data.cn0 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [offsets_msb]
    data.offsets_msb = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [carrier_msb]
    data.carrier_msb = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [obs_info]
    data.obs_info = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [code_offset_lsb]
    data.code_offset_lsb = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [carrier_lsb]
    data.carrier_lsb = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [doppler_offset_lsb]
    data.doppler_offset_lsb = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/MeasEpochChannelType2';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '204efbe40d2e56cd210b50c219f3a400';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MeasEpochChannelType2 block
       
    uint8  type
    uint8  lock_time          # s
    uint8  cn0                # 0.25 dB-Hz
    uint8  offsets_msb        # 65.536 m or 65.536 Hz
    int8   carrier_msb        # 65.536 cycles
    uint8  obs_info
    uint16 code_offset_lsb    # 0.001 m
    uint16 carrier_lsb        # 0.001 cycles
    uint16 doppler_offset_lsb # 0.0001 Hz
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MeasEpochChannelType2(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.lock_time !== undefined) {
      resolved.lock_time = msg.lock_time;
    }
    else {
      resolved.lock_time = 0
    }

    if (msg.cn0 !== undefined) {
      resolved.cn0 = msg.cn0;
    }
    else {
      resolved.cn0 = 0
    }

    if (msg.offsets_msb !== undefined) {
      resolved.offsets_msb = msg.offsets_msb;
    }
    else {
      resolved.offsets_msb = 0
    }

    if (msg.carrier_msb !== undefined) {
      resolved.carrier_msb = msg.carrier_msb;
    }
    else {
      resolved.carrier_msb = 0
    }

    if (msg.obs_info !== undefined) {
      resolved.obs_info = msg.obs_info;
    }
    else {
      resolved.obs_info = 0
    }

    if (msg.code_offset_lsb !== undefined) {
      resolved.code_offset_lsb = msg.code_offset_lsb;
    }
    else {
      resolved.code_offset_lsb = 0
    }

    if (msg.carrier_lsb !== undefined) {
      resolved.carrier_lsb = msg.carrier_lsb;
    }
    else {
      resolved.carrier_lsb = 0
    }

    if (msg.doppler_offset_lsb !== undefined) {
      resolved.doppler_offset_lsb = msg.doppler_offset_lsb;
    }
    else {
      resolved.doppler_offset_lsb = 0
    }

    return resolved;
    }
};

module.exports = MeasEpochChannelType2;
