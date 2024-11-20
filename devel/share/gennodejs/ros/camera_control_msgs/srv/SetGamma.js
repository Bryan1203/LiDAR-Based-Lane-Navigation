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

class SetGammaRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_gamma = null;
    }
    else {
      if (initObj.hasOwnProperty('target_gamma')) {
        this.target_gamma = initObj.target_gamma
      }
      else {
        this.target_gamma = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetGammaRequest
    // Serialize message field [target_gamma]
    bufferOffset = _serializer.float32(obj.target_gamma, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetGammaRequest
    let len;
    let data = new SetGammaRequest(null);
    // Deserialize message field [target_gamma]
    data.target_gamma = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetGammaRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0a9682c360c7e6178269a064cdf9a0c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The target gamma correction of pixel intensity.
    # Adjusts the brightness of the pixel values output by the camera's sensor
    # to account for a non-linearity in the human perception of brightness or
    # of the display system (such as CRT).
    float32 target_gamma
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetGammaRequest(null);
    if (msg.target_gamma !== undefined) {
      resolved.target_gamma = msg.target_gamma;
    }
    else {
      resolved.target_gamma = 0.0
    }

    return resolved;
    }
};

class SetGammaResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reached_gamma = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('reached_gamma')) {
        this.reached_gamma = initObj.reached_gamma
      }
      else {
        this.reached_gamma = 0.0;
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
    // Serializes a message object of type SetGammaResponse
    // Serialize message field [reached_gamma]
    bufferOffset = _serializer.float32(obj.reached_gamma, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetGammaResponse
    let len;
    let data = new SetGammaResponse(null);
    // Deserialize message field [reached_gamma]
    data.reached_gamma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetGammaResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f5a47cc339c639a71a650af05aed3b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Exact match can not always be reached
    float32 reached_gamma
    bool success
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetGammaResponse(null);
    if (msg.reached_gamma !== undefined) {
      resolved.reached_gamma = msg.reached_gamma;
    }
    else {
      resolved.reached_gamma = 0.0
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
  Request: SetGammaRequest,
  Response: SetGammaResponse,
  md5sum() { return 'd071cb93e1f4c5737ce40debcd7942a5'; },
  datatype() { return 'camera_control_msgs/SetGamma'; }
};
