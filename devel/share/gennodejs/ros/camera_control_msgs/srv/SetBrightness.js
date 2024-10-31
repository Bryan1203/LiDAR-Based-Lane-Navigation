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

class SetBrightnessRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_brightness = null;
      this.brightness_continuous = null;
      this.exposure_auto = null;
      this.gain_auto = null;
    }
    else {
      if (initObj.hasOwnProperty('target_brightness')) {
        this.target_brightness = initObj.target_brightness
      }
      else {
        this.target_brightness = 0;
      }
      if (initObj.hasOwnProperty('brightness_continuous')) {
        this.brightness_continuous = initObj.brightness_continuous
      }
      else {
        this.brightness_continuous = false;
      }
      if (initObj.hasOwnProperty('exposure_auto')) {
        this.exposure_auto = initObj.exposure_auto
      }
      else {
        this.exposure_auto = false;
      }
      if (initObj.hasOwnProperty('gain_auto')) {
        this.gain_auto = initObj.gain_auto
      }
      else {
        this.gain_auto = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetBrightnessRequest
    // Serialize message field [target_brightness]
    bufferOffset = _serializer.int32(obj.target_brightness, buffer, bufferOffset);
    // Serialize message field [brightness_continuous]
    bufferOffset = _serializer.bool(obj.brightness_continuous, buffer, bufferOffset);
    // Serialize message field [exposure_auto]
    bufferOffset = _serializer.bool(obj.exposure_auto, buffer, bufferOffset);
    // Serialize message field [gain_auto]
    bufferOffset = _serializer.bool(obj.gain_auto, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetBrightnessRequest
    let len;
    let data = new SetBrightnessRequest(null);
    // Deserialize message field [target_brightness]
    data.target_brightness = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [brightness_continuous]
    data.brightness_continuous = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [exposure_auto]
    data.exposure_auto = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gain_auto]
    data.gain_auto = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 7;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetBrightnessRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1f8cc426d8d4e41959f6306162a714c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The target brightness, which is average intensity values of the images.
    # It depends the exposure time as well as the gain setting.
    int32 target_brightness
    
    # The brightness_continuous flag controls the auto brightness function.
    # If it is set to false, the given brightness will only be reached once.
    # Hence changing light conditions lead to changing brightness values.
    # If it is set to true, the given brightness will be reached continuously,
    # trying to adapt to changing light conditions. The 'brightness_contunuous'
    # mode is is only possible for values in the possible auto range of the arena
    # API which is e.g. [50 - 205] for acA2500-14um and acA1920-40gm
    bool brightness_continuous
    
    # If the camera should try reach or keep the desired brightness, hence adapting
    # to changing light conditions, at least one of the following flags MUST be set.
    # If both are set, the interface will use the profile that tries to keep the
    # gain at minimum to reduce white noise.
    # 'exposure_auto' will adapt the exposure time to reach the brightness, wheras
    # 'gain_auto' does so by adapting the gain.
    bool exposure_auto
    bool gain_auto
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetBrightnessRequest(null);
    if (msg.target_brightness !== undefined) {
      resolved.target_brightness = msg.target_brightness;
    }
    else {
      resolved.target_brightness = 0
    }

    if (msg.brightness_continuous !== undefined) {
      resolved.brightness_continuous = msg.brightness_continuous;
    }
    else {
      resolved.brightness_continuous = false
    }

    if (msg.exposure_auto !== undefined) {
      resolved.exposure_auto = msg.exposure_auto;
    }
    else {
      resolved.exposure_auto = false
    }

    if (msg.gain_auto !== undefined) {
      resolved.gain_auto = msg.gain_auto;
    }
    else {
      resolved.gain_auto = false
    }

    return resolved;
    }
};

class SetBrightnessResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reached_brightness = null;
      this.reached_exposure_time = null;
      this.reached_gain_value = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('reached_brightness')) {
        this.reached_brightness = initObj.reached_brightness
      }
      else {
        this.reached_brightness = 0;
      }
      if (initObj.hasOwnProperty('reached_exposure_time')) {
        this.reached_exposure_time = initObj.reached_exposure_time
      }
      else {
        this.reached_exposure_time = 0.0;
      }
      if (initObj.hasOwnProperty('reached_gain_value')) {
        this.reached_gain_value = initObj.reached_gain_value
      }
      else {
        this.reached_gain_value = 0.0;
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
    // Serializes a message object of type SetBrightnessResponse
    // Serialize message field [reached_brightness]
    bufferOffset = _serializer.int32(obj.reached_brightness, buffer, bufferOffset);
    // Serialize message field [reached_exposure_time]
    bufferOffset = _serializer.float32(obj.reached_exposure_time, buffer, bufferOffset);
    // Serialize message field [reached_gain_value]
    bufferOffset = _serializer.float32(obj.reached_gain_value, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetBrightnessResponse
    let len;
    let data = new SetBrightnessResponse(null);
    // Deserialize message field [reached_brightness]
    data.reached_brightness = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [reached_exposure_time]
    data.reached_exposure_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [reached_gain_value]
    data.reached_gain_value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetBrightnessResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '62110aff39d46cf6a4ab77f09896582b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # Exact match can not always be reached
    int32 reached_brightness
    float32 reached_exposure_time
    float32 reached_gain_value
    
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetBrightnessResponse(null);
    if (msg.reached_brightness !== undefined) {
      resolved.reached_brightness = msg.reached_brightness;
    }
    else {
      resolved.reached_brightness = 0
    }

    if (msg.reached_exposure_time !== undefined) {
      resolved.reached_exposure_time = msg.reached_exposure_time;
    }
    else {
      resolved.reached_exposure_time = 0.0
    }

    if (msg.reached_gain_value !== undefined) {
      resolved.reached_gain_value = msg.reached_gain_value;
    }
    else {
      resolved.reached_gain_value = 0.0
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
  Request: SetBrightnessRequest,
  Response: SetBrightnessResponse,
  md5sum() { return '36481c45e92096457660dc69b68eb6b1'; },
  datatype() { return 'camera_control_msgs/SetBrightness'; }
};
