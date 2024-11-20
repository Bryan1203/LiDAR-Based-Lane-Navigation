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

class VectorInfoCart {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.nr_sv = null;
      this.error = null;
      this.mode = null;
      this.misc = null;
      this.delta_x = null;
      this.delta_y = null;
      this.delta_z = null;
      this.delta_vx = null;
      this.delta_vy = null;
      this.delta_vz = null;
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
      if (initObj.hasOwnProperty('delta_x')) {
        this.delta_x = initObj.delta_x
      }
      else {
        this.delta_x = 0.0;
      }
      if (initObj.hasOwnProperty('delta_y')) {
        this.delta_y = initObj.delta_y
      }
      else {
        this.delta_y = 0.0;
      }
      if (initObj.hasOwnProperty('delta_z')) {
        this.delta_z = initObj.delta_z
      }
      else {
        this.delta_z = 0.0;
      }
      if (initObj.hasOwnProperty('delta_vx')) {
        this.delta_vx = initObj.delta_vx
      }
      else {
        this.delta_vx = 0.0;
      }
      if (initObj.hasOwnProperty('delta_vy')) {
        this.delta_vy = initObj.delta_vy
      }
      else {
        this.delta_vy = 0.0;
      }
      if (initObj.hasOwnProperty('delta_vz')) {
        this.delta_vz = initObj.delta_vz
      }
      else {
        this.delta_vz = 0.0;
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
    // Serializes a message object of type VectorInfoCart
    // Serialize message field [nr_sv]
    bufferOffset = _serializer.uint8(obj.nr_sv, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.uint8(obj.error, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [misc]
    bufferOffset = _serializer.uint8(obj.misc, buffer, bufferOffset);
    // Serialize message field [delta_x]
    bufferOffset = _serializer.float64(obj.delta_x, buffer, bufferOffset);
    // Serialize message field [delta_y]
    bufferOffset = _serializer.float64(obj.delta_y, buffer, bufferOffset);
    // Serialize message field [delta_z]
    bufferOffset = _serializer.float64(obj.delta_z, buffer, bufferOffset);
    // Serialize message field [delta_vx]
    bufferOffset = _serializer.float32(obj.delta_vx, buffer, bufferOffset);
    // Serialize message field [delta_vy]
    bufferOffset = _serializer.float32(obj.delta_vy, buffer, bufferOffset);
    // Serialize message field [delta_vz]
    bufferOffset = _serializer.float32(obj.delta_vz, buffer, bufferOffset);
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
    //deserializes a message object of type VectorInfoCart
    let len;
    let data = new VectorInfoCart(null);
    // Deserialize message field [nr_sv]
    data.nr_sv = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [misc]
    data.misc = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [delta_x]
    data.delta_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_y]
    data.delta_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_z]
    data.delta_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_vx]
    data.delta_vx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [delta_vy]
    data.delta_vy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [delta_vz]
    data.delta_vz = _deserializer.float32(buffer, bufferOffset);
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
    return 'septentrio_gnss_driver/VectorInfoCart';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd072946ecd7f4319790a07cf534d27c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new VectorInfoCart(null);
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

    if (msg.delta_x !== undefined) {
      resolved.delta_x = msg.delta_x;
    }
    else {
      resolved.delta_x = 0.0
    }

    if (msg.delta_y !== undefined) {
      resolved.delta_y = msg.delta_y;
    }
    else {
      resolved.delta_y = 0.0
    }

    if (msg.delta_z !== undefined) {
      resolved.delta_z = msg.delta_z;
    }
    else {
      resolved.delta_z = 0.0
    }

    if (msg.delta_vx !== undefined) {
      resolved.delta_vx = msg.delta_vx;
    }
    else {
      resolved.delta_vx = 0.0
    }

    if (msg.delta_vy !== undefined) {
      resolved.delta_vy = msg.delta_vy;
    }
    else {
      resolved.delta_vy = 0.0
    }

    if (msg.delta_vz !== undefined) {
      resolved.delta_vz = msg.delta_vz;
    }
    else {
      resolved.delta_vz = 0.0
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

module.exports = VectorInfoCart;
