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

class SetExposureRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_exposure = null;
    }
    else {
      if (initObj.hasOwnProperty('target_exposure')) {
        this.target_exposure = initObj.target_exposure
      }
      else {
        this.target_exposure = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetExposureRequest
    // Serialize message field [target_exposure]
    bufferOffset = _serializer.float32(obj.target_exposure, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetExposureRequest
    let len;
    let data = new SetExposureRequest(null);
    // Deserialize message field [target_exposure]
    data.target_exposure = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetExposureRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '881807b9f62e6919695085297c872d67';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The target exposure time measured in microseconds.
    # If the limits were exceeded, the desired exposure time will be truncated.
    float32 target_exposure
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetExposureRequest(null);
    if (msg.target_exposure !== undefined) {
      resolved.target_exposure = msg.target_exposure;
    }
    else {
      resolved.target_exposure = 0.0
    }

    return resolved;
    }
};

class SetExposureResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reached_exposure = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('reached_exposure')) {
        this.reached_exposure = initObj.reached_exposure
      }
      else {
        this.reached_exposure = 0.0;
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
    // Serializes a message object of type SetExposureResponse
    // Serialize message field [reached_exposure]
    bufferOffset = _serializer.float32(obj.reached_exposure, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetExposureResponse
    let len;
    let data = new SetExposureResponse(null);
    // Deserialize message field [reached_exposure]
    data.reached_exposure = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetExposureResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f0624b49b89ef97572f8db48c91665b2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Exact match can not always be reached
    float32 reached_exposure
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetExposureResponse(null);
    if (msg.reached_exposure !== undefined) {
      resolved.reached_exposure = msg.reached_exposure;
    }
    else {
      resolved.reached_exposure = 0.0
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
  Request: SetExposureRequest,
  Response: SetExposureResponse,
  md5sum() { return '07036591479e56cbaa8b13d231a2c756'; },
  datatype() { return 'camera_control_msgs/SetExposure'; }
};
