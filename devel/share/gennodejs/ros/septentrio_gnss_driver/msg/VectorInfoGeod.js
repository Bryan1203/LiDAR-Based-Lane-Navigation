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

class VectorInfoGeod {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.nr_sv = null;
      this.error = null;
      this.mode = null;
      this.misc = null;
      this.delta_east = null;
      this.delta_north = null;
      this.delta_up = null;
      this.delta_ve = null;
      this.delta_vn = null;
      this.delta_vu = null;
      this.azimuth = null;
      this.elevation = null;
      this.reference_id = null;
      this.corr_age = null;
      this.signal_info = null;
    }
    else {
      if (initObj.hasOwnProperty('nr_sv')) {
        this.nr_sv = initObj.nr_sv
      }
      else {
        this.nr_sv = 0;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('misc')) {
        this.misc = initObj.misc
      }
      else {
        this.misc = 0;
      }
      if (initObj.hasOwnProperty('delta_east')) {
        this.delta_east = initObj.delta_east
      }
      else {
        this.delta_east = 0.0;
      }
      if (initObj.hasOwnProperty('delta_north')) {
        this.delta_north = initObj.delta_north
      }
      else {
        this.delta_north = 0.0;
      }
      if (initObj.hasOwnProperty('delta_up')) {
        this.delta_up = initObj.delta_up
      }
      else {
        this.delta_up = 0.0;
      }
      if (initObj.hasOwnProperty('delta_ve')) {
        this.delta_ve = initObj.delta_ve
      }
      else {
        this.delta_ve = 0.0;
      }
      if (initObj.hasOwnProperty('delta_vn')) {
        this.delta_vn = initObj.delta_vn
      }
      else {
        this.delta_vn = 0.0;
      }
      if (initObj.hasOwnProperty('delta_vu')) {
        this.delta_vu = initObj.delta_vu
      }
      else {
        this.delta_vu = 0.0;
      }
      if (initObj.hasOwnProperty('azimuth')) {
        this.azimuth = initObj.azimuth
      }
      else {
        this.azimuth = 0;
      }
      if (initObj.hasOwnProperty('elevation')) {
        this.elevation = initObj.elevation
      }
      else {
        this.elevation = 0;
      }
      if (initObj.hasOwnProperty('reference_id')) {
        this.reference_id = initObj.reference_id
      }
      else {
        this.reference_id = 0;
      }
      if (initObj.hasOwnProperty('corr_age')) {
        this.corr_age = initObj.corr_age
      }
      else {
        this.corr_age = 0;
      }
      if (initObj.hasOwnProperty('signal_info')) {
        this.signal_info = initObj.signal_info
      }
      else {
        this.signal_info = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VectorInfoGeod
    // Serialize message field [nr_sv]
    bufferOffset = _serializer.uint8(obj.nr_sv, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [misc]
    bufferOffset = _serializer.uint8(obj.misc, buffer, bufferOffset);
    // Serialize message field [delta_east]
    bufferOffset = _serializer.float64(obj.delta_east, buffer, bufferOffset);
    // Serialize message field [delta_north]
    bufferOffset = _serializer.float64(obj.delta_north, buffer, bufferOffset);
    // Serialize message field [delta_up]
    bufferOffset = _serializer.float64(obj.delta_up, buffer, bufferOffset);
    // Serialize message field [delta_ve]
    bufferOffset = _serializer.float32(obj.delta_ve, buffer, bufferOffset);
    // Serialize message field [delta_vn]
    bufferOffset = _serializer.float32(obj.delta_vn, buffer, bufferOffset);
    // Serialize message field [delta_vu]
    bufferOffset = _serializer.float32(obj.delta_vu, buffer, bufferOffset);
    // Serialize message field [azimuth]
    bufferOffset = _serializer.uint16(obj.azimuth, buffer, bufferOffset);
    // Serialize message field [elevation]
    bufferOffset = _serializer.int16(obj.elevation, buffer, bufferOffset);
    // Serialize message field [reference_id]
    bufferOffset = _serializer.uint16(obj.reference_id, buffer, bufferOffset);
    // Serialize message field [corr_age]
    bufferOffset = _serializer.uint16(obj.corr_age, buffer, bufferOffset);
    // Serialize message field [signal_info]
    bufferOffset = _serializer.uint32(obj.signal_info, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VectorInfoGeod
    let len;
    let data = new VectorInfoGeod(null);
    // Deserialize message field [nr_sv]
    data.nr_sv = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [misc]
    data.misc = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [delta_east]
    data.delta_east = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_north]
    data.delta_north = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_up]
    data.delta_up = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_ve]
    data.delta_ve = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [delta_vn]
    data.delta_vn = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [delta_vu]
    data.delta_vu = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [azimuth]
    data.azimuth = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [elevation]
    data.elevation = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [reference_id]
    data.reference_id = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [corr_age]
    data.corr_age = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [signal_info]
    data.signal_info = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'septentrio_gnss_driver/VectorInfoGeod';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dbd3c8ede9fdca23023a59b11ba2b072';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # VectorInfoGeod block
       
    uint8   nr_sv
    uint8   error
    uint8   mode
    uint8   misc
    float64 delta_east   # m
    float64 delta_north  # m
    float64 delta_up     # m
    float32 delta_ve     # m
    float32 delta_vn     # m
    float32 delta_vu     # m
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
    const resolved = new VectorInfoGeod(null);
    if (msg.nr_sv !== undefined) {
      resolved.nr_sv = msg.nr_sv;
    }
    else {
      resolved.nr_sv = 0
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.misc !== undefined) {
      resolved.misc = msg.misc;
    }
    else {
      resolved.misc = 0
    }

    if (msg.delta_east !== undefined) {
      resolved.delta_east = msg.delta_east;
    }
    else {
      resolved.delta_east = 0.0
    }

    if (msg.delta_north !== undefined) {
      resolved.delta_north = msg.delta_north;
    }
    else {
      resolved.delta_north = 0.0
    }

    if (msg.delta_up !== undefined) {
      resolved.delta_up = msg.delta_up;
    }
    else {
      resolved.delta_up = 0.0
    }

    if (msg.delta_ve !== undefined) {
      resolved.delta_ve = msg.delta_ve;
    }
    else {
      resolved.delta_ve = 0.0
    }

    if (msg.delta_vn !== undefined) {
      resolved.delta_vn = msg.delta_vn;
    }
    else {
      resolved.delta_vn = 0.0
    }

    if (msg.delta_vu !== undefined) {
      resolved.delta_vu = msg.delta_vu;
    }
    else {
      resolved.delta_vu = 0.0
    }

    if (msg.azimuth !== undefined) {
      resolved.azimuth = msg.azimuth;
    }
    else {
      resolved.azimuth = 0
    }

    if (msg.elevation !== undefined) {
      resolved.elevation = msg.elevation;
    }
    else {
      resolved.elevation = 0
    }

    if (msg.reference_id !== undefined) {
      resolved.reference_id = msg.reference_id;
    }
    else {
      resolved.reference_id = 0
    }

    if (msg.corr_age !== undefined) {
      resolved.corr_age = msg.corr_age;
    }
    else {
      resolved.corr_age = 0
    }

    if (msg.signal_info !== undefined) {
      resolved.signal_info = msg.signal_info;
    }
    else {
      resolved.signal_info = 0
    }

    return resolved;
    }
};

module.exports = VectorInfoGeod;
