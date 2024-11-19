// Auto-generated. Do not edit!

// (in-package septentrio_gnss_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MeasEpochChannelType2 = require('./MeasEpochChannelType2.js');

//-----------------------------------------------------------

class MeasEpochChannelType1 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rx_channel = null;
      this.type = null;
      this.sv_id = null;
      this.misc = null;
      this.code_lsb = null;
      this.doppler = null;
      this.carrier_lsb = null;
      this.carrier_msb = null;
      this.cn0 = null;
      this.lock_time = null;
      this.obs_info = null;
      this.n2 = null;
      this.type2 = null;
    }
    else {
      if (initObj.hasOwnProperty('rx_channel')) {
        this.rx_channel = initObj.rx_channel
      }
      else {
        this.rx_channel = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('sv_id')) {
        this.sv_id = initObj.sv_id
      }
      else {
        this.sv_id = 0;
      }
      if (initObj.hasOwnProperty('misc')) {
        this.misc = initObj.misc
      }
      else {
        this.misc = 0;
      }
      if (initObj.hasOwnProperty('code_lsb')) {
        this.code_lsb = initObj.code_lsb
      }
      else {
        this.code_lsb = 0;
      }
      if (initObj.hasOwnProperty('doppler')) {
        this.doppler = initObj.doppler
      }
      else {
        this.doppler = 0;
      }
      if (initObj.hasOwnProperty('carrier_lsb')) {
        this.carrier_lsb = initObj.carrier_lsb
      }
      else {
        this.carrier_lsb = 0;
      }
      if (initObj.hasOwnProperty('carrier_msb')) {
        this.carrier_msb = initObj.carrier_msb
      }
      else {
        this.carrier_msb = 0;
      }
      if (initObj.hasOwnProperty('cn0')) {
        this.cn0 = initObj.cn0
      }
      else {
        this.cn0 = 0;
      }
      if (initObj.hasOwnProperty('lock_time')) {
        this.lock_time = initObj.lock_time
      }
      else {
        this.lock_time = 0;
      }
      if (initObj.hasOwnProperty('obs_info')) {
        this.obs_info = initObj.obs_info
      }
      else {
        this.obs_info = 0;
      }
      if (initObj.hasOwnProperty('n2')) {
        this.n2 = initObj.n2
      }
      else {
        this.n2 = 0;
      }
      if (initObj.hasOwnProperty('type2')) {
        this.type2 = initObj.type2
      }
      else {
        this.type2 = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MeasEpochChannelType1
    // Serialize message field [rx_channel]
    bufferOffset = _serializer.uint8(obj.rx_channel, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.uint8(obj.type, buffer, bufferOffset);
    // Serialize message field [sv_id]
    bufferOffset = _serializer.uint8(obj.sv_id, buffer, bufferOffset);
    // Serialize message field [misc]
    bufferOffset = _serializer.uint8(obj.misc, buffer, bufferOffset);
    // Serialize message field [code_lsb]
    bufferOffset = _serializer.uint32(obj.code_lsb, buffer, bufferOffset);
    // Serialize message field [doppler]
    bufferOffset = _serializer.int32(obj.doppler, buffer, bufferOffset);
    // Serialize message field [carrier_lsb]
    bufferOffset = _serializer.uint16(obj.carrier_lsb, buffer, bufferOffset);
    // Serialize message field [carrier_msb]
    bufferOffset = _serializer.int8(obj.carrier_msb, buffer, bufferOffset);
    // Serialize message field [cn0]
    bufferOffset = _serializer.uint8(obj.cn0, buffer, bufferOffset);
    // Serialize message field [lock_time]
    bufferOffset = _serializer.uint16(obj.lock_time, buffer, bufferOffset);
    // Serialize message field [obs_info]
    bufferOffset = _serializer.uint8(obj.obs_info, buffer, bufferOffset);
    // Serialize message field [n2]
    bufferOffset = _serializer.uint8(obj.n2, buffer, bufferOffset);
    // Serialize message field [type2]
    // Serialize the length for message field [type2]
    bufferOffset = _serializer.uint32(obj.type2.length, buffer, bufferOffset);
    obj.type2.forEach((val) => {
      bufferOffset = MeasEpochChannelType2.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MeasEpochChannelType1
    let len;
    let data = new MeasEpochChannelType1(null);
    // Deserialize message field [rx_channel]
    data.rx_channel = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sv_id]
    data.sv_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [misc]
    data.misc = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [code_lsb]
    data.code_lsb = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [doppler]
    data.doppler = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [carrier_lsb]
    data.carrier_lsb = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [carrier_msb]
    data.carrier_msb = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [cn0]
    data.cn0 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [lock_time]
    data.lock_time = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [obs_info]
    data.obs_info = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [n2]
    data.n2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [type2]
    // Deserialize array length for message field [type2]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.type2 = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.type2[i] = MeasEpochChannelType2.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.type2.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/MeasEpochChannelType1';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6ec5531728aded72fdbf7e7550cabd87';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MeasEpochChannelType1 block
       
    uint8  rx_channel
    uint8  type
    uint8  sv_id
    uint8  misc        # 4294967.296 m
    uint32 code_lsb    # 0.001 m
    int32  doppler     # 0.0001 Hz
    uint16 carrier_lsb # 0.001 cycles
    int8   carrier_msb # 65.536 cycles
    uint8  cn0         # 0.25 dB-Hz
    uint16 lock_time
    uint8  obs_info
    uint8  n2
    
    MeasEpochChannelType2[] type2
    ================================================================================
    MSG: septentrio_gnss_driver/MeasEpochChannelType2
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
    const resolved = new MeasEpochChannelType1(null);
    if (msg.rx_channel !== undefined) {
      resolved.rx_channel = msg.rx_channel;
    }
    else {
      resolved.rx_channel = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.sv_id !== undefined) {
      resolved.sv_id = msg.sv_id;
    }
    else {
      resolved.sv_id = 0
    }

    if (msg.misc !== undefined) {
      resolved.misc = msg.misc;
    }
    else {
      resolved.misc = 0
    }

    if (msg.code_lsb !== undefined) {
      resolved.code_lsb = msg.code_lsb;
    }
    else {
      resolved.code_lsb = 0
    }

    if (msg.doppler !== undefined) {
      resolved.doppler = msg.doppler;
    }
    else {
      resolved.doppler = 0
    }

    if (msg.carrier_lsb !== undefined) {
      resolved.carrier_lsb = msg.carrier_lsb;
    }
    else {
      resolved.carrier_lsb = 0
    }

    if (msg.carrier_msb !== undefined) {
      resolved.carrier_msb = msg.carrier_msb;
    }
    else {
      resolved.carrier_msb = 0
    }

    if (msg.cn0 !== undefined) {
      resolved.cn0 = msg.cn0;
    }
    else {
      resolved.cn0 = 0
    }

    if (msg.lock_time !== undefined) {
      resolved.lock_time = msg.lock_time;
    }
    else {
      resolved.lock_time = 0
    }

    if (msg.obs_info !== undefined) {
      resolved.obs_info = msg.obs_info;
    }
    else {
      resolved.obs_info = 0
    }

    if (msg.n2 !== undefined) {
      resolved.n2 = msg.n2;
    }
    else {
      resolved.n2 = 0
    }

    if (msg.type2 !== undefined) {
      resolved.type2 = new Array(msg.type2.length);
      for (let i = 0; i < resolved.type2.length; ++i) {
        resolved.type2[i] = MeasEpochChannelType2.Resolve(msg.type2[i]);
      }
    }
    else {
      resolved.type2 = []
    }

    return resolved;
    }
};

module.exports = MeasEpochChannelType1;
