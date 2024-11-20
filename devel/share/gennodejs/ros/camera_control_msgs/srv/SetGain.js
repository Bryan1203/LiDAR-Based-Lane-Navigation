// Auto-generated. Do not edit!

// (in-package camera_control_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetGainRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_gain = null;
    }
    else {
      if (initObj.hasOwnProperty('target_gain')) {
        this.target_gain = initObj.target_gain
      }
      else {
        this.target_gain = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetGainRequest
    // Serialize message field [target_gain]
    bufferOffset = _serializer.float32(obj.target_gain, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetGainRequest
    let len;
    let data = new SetGainRequest(null);
    // Deserialize message field [target_gain]
    data.target_gain = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetGainRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5e755c5a3e141e0e1df554ec3425596f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The target gain in percent of the maximal value the camera supports.
    # For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
    # called 'device specific units'.
    float32 target_gain
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetGainRequest(null);
    if (msg.target_gain !== undefined) {
      resolved.target_gain = msg.target_gain;
    }
    else {
      resolved.target_gain = 0.0
    }

    return resolved;
    }
};

class SetGainResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reached_gain = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('reached_gain')) {
        this.reached_gain = initObj.reached_gain
      }
      else {
        this.reached_gain = 0.0;
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetGainResponse
    // Serialize message field [reached_gain]
    bufferOffset = _serializer.float32(obj.reached_gain, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetGainResponse
    let len;
    let data = new SetGainResponse(null);
    // Deserialize message field [reached_gain]
    data.reached_gain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetGainResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '885668c9ebfd53815071a529456c4b42';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Exact match can not always be reached
    float32 reached_gain
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetGainResponse(null);
    if (msg.reached_gain !== undefined) {
      resolved.reached_gain = msg.reached_gain;
    }
    else {
      resolved.reached_gain = 0.0
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SetGainRequest,
  Response: SetGainResponse,
  md5sum() { return '20482afb1d808bae25db22114b0a747f'; },
  datatype() { return 'camera_control_msgs/SetGain'; }
};
