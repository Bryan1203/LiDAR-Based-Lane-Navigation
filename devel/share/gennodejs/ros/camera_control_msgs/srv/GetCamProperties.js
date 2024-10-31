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

class GetCamPropertiesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCamPropertiesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCamPropertiesRequest
    let len;
    let data = new GetCamPropertiesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/GetCamPropertiesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This service returns the current camera properties as well as the limits
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCamPropertiesRequest(null);
    return resolved;
    }
};

class GetCamPropertiesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.is_sleeping = null;
      this.device_user_id = null;
      this.min_binning_x = null;
      this.max_binning_x = null;
      this.current_binning_x = null;
      this.min_binning_y = null;
      this.max_binning_y = null;
      this.current_binning_y = null;
      this.max_framerate = null;
      this.current_framerate = null;
      this.min_exposure = null;
      this.max_exposure = null;
      this.current_exposure = null;
      this.min_gain_in_cam_units = null;
      this.max_gain_in_cam_units = null;
      this.current_gain_in_cam_units = null;
      this.min_gain = null;
      this.max_gain = null;
      this.current_gain = null;
      this.min_gamma = null;
      this.max_gamma = null;
      this.current_gamma = null;
      this.brightness_continuous = null;
      this.gain_auto = null;
      this.exposure_auto = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('is_sleeping')) {
        this.is_sleeping = initObj.is_sleeping
      }
      else {
        this.is_sleeping = false;
      }
      if (initObj.hasOwnProperty('device_user_id')) {
        this.device_user_id = initObj.device_user_id
      }
      else {
        this.device_user_id = '';
      }
      if (initObj.hasOwnProperty('min_binning_x')) {
        this.min_binning_x = initObj.min_binning_x
      }
      else {
        this.min_binning_x = 0;
      }
      if (initObj.hasOwnProperty('max_binning_x')) {
        this.max_binning_x = initObj.max_binning_x
      }
      else {
        this.max_binning_x = 0;
      }
      if (initObj.hasOwnProperty('current_binning_x')) {
        this.current_binning_x = initObj.current_binning_x
      }
      else {
        this.current_binning_x = 0;
      }
      if (initObj.hasOwnProperty('min_binning_y')) {
        this.min_binning_y = initObj.min_binning_y
      }
      else {
        this.min_binning_y = 0;
      }
      if (initObj.hasOwnProperty('max_binning_y')) {
        this.max_binning_y = initObj.max_binning_y
      }
      else {
        this.max_binning_y = 0;
      }
      if (initObj.hasOwnProperty('current_binning_y')) {
        this.current_binning_y = initObj.current_binning_y
      }
      else {
        this.current_binning_y = 0;
      }
      if (initObj.hasOwnProperty('max_framerate')) {
        this.max_framerate = initObj.max_framerate
      }
      else {
        this.max_framerate = 0.0;
      }
      if (initObj.hasOwnProperty('current_framerate')) {
        this.current_framerate = initObj.current_framerate
      }
      else {
        this.current_framerate = 0.0;
      }
      if (initObj.hasOwnProperty('min_exposure')) {
        this.min_exposure = initObj.min_exposure
      }
      else {
        this.min_exposure = 0.0;
      }
      if (initObj.hasOwnProperty('max_exposure')) {
        this.max_exposure = initObj.max_exposure
      }
      else {
        this.max_exposure = 0.0;
      }
      if (initObj.hasOwnProperty('current_exposure')) {
        this.current_exposure = initObj.current_exposure
      }
      else {
        this.current_exposure = 0.0;
      }
      if (initObj.hasOwnProperty('min_gain_in_cam_units')) {
        this.min_gain_in_cam_units = initObj.min_gain_in_cam_units
      }
      else {
        this.min_gain_in_cam_units = 0.0;
      }
      if (initObj.hasOwnProperty('max_gain_in_cam_units')) {
        this.max_gain_in_cam_units = initObj.max_gain_in_cam_units
      }
      else {
        this.max_gain_in_cam_units = 0.0;
      }
      if (initObj.hasOwnProperty('current_gain_in_cam_units')) {
        this.current_gain_in_cam_units = initObj.current_gain_in_cam_units
      }
      else {
        this.current_gain_in_cam_units = 0.0;
      }
      if (initObj.hasOwnProperty('min_gain')) {
        this.min_gain = initObj.min_gain
      }
      else {
        this.min_gain = 0.0;
      }
      if (initObj.hasOwnProperty('max_gain')) {
        this.max_gain = initObj.max_gain
      }
      else {
        this.max_gain = 0.0;
      }
      if (initObj.hasOwnProperty('current_gain')) {
        this.current_gain = initObj.current_gain
      }
      else {
        this.current_gain = 0.0;
      }
      if (initObj.hasOwnProperty('min_gamma')) {
        this.min_gamma = initObj.min_gamma
      }
      else {
        this.min_gamma = 0.0;
      }
      if (initObj.hasOwnProperty('max_gamma')) {
        this.max_gamma = initObj.max_gamma
      }
      else {
        this.max_gamma = 0.0;
      }
      if (initObj.hasOwnProperty('current_gamma')) {
        this.current_gamma = initObj.current_gamma
      }
      else {
        this.current_gamma = 0.0;
      }
      if (initObj.hasOwnProperty('brightness_continuous')) {
        this.brightness_continuous = initObj.brightness_continuous
      }
      else {
        this.brightness_continuous = false;
      }
      if (initObj.hasOwnProperty('gain_auto')) {
        this.gain_auto = initObj.gain_auto
      }
      else {
        this.gain_auto = false;
      }
      if (initObj.hasOwnProperty('exposure_auto')) {
        this.exposure_auto = initObj.exposure_auto
      }
      else {
        this.exposure_auto = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCamPropertiesResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [is_sleeping]
    bufferOffset = _serializer.bool(obj.is_sleeping, buffer, bufferOffset);
    // Serialize message field [device_user_id]
    bufferOffset = _serializer.string(obj.device_user_id, buffer, bufferOffset);
    // Serialize message field [min_binning_x]
    bufferOffset = _serializer.int32(obj.min_binning_x, buffer, bufferOffset);
    // Serialize message field [max_binning_x]
    bufferOffset = _serializer.int32(obj.max_binning_x, buffer, bufferOffset);
    // Serialize message field [current_binning_x]
    bufferOffset = _serializer.int32(obj.current_binning_x, buffer, bufferOffset);
    // Serialize message field [min_binning_y]
    bufferOffset = _serializer.int32(obj.min_binning_y, buffer, bufferOffset);
    // Serialize message field [max_binning_y]
    bufferOffset = _serializer.int32(obj.max_binning_y, buffer, bufferOffset);
    // Serialize message field [current_binning_y]
    bufferOffset = _serializer.int32(obj.current_binning_y, buffer, bufferOffset);
    // Serialize message field [max_framerate]
    bufferOffset = _serializer.float32(obj.max_framerate, buffer, bufferOffset);
    // Serialize message field [current_framerate]
    bufferOffset = _serializer.float32(obj.current_framerate, buffer, bufferOffset);
    // Serialize message field [min_exposure]
    bufferOffset = _serializer.float32(obj.min_exposure, buffer, bufferOffset);
    // Serialize message field [max_exposure]
    bufferOffset = _serializer.float32(obj.max_exposure, buffer, bufferOffset);
    // Serialize message field [current_exposure]
    bufferOffset = _serializer.float32(obj.current_exposure, buffer, bufferOffset);
    // Serialize message field [min_gain_in_cam_units]
    bufferOffset = _serializer.float32(obj.min_gain_in_cam_units, buffer, bufferOffset);
    // Serialize message field [max_gain_in_cam_units]
    bufferOffset = _serializer.float32(obj.max_gain_in_cam_units, buffer, bufferOffset);
    // Serialize message field [current_gain_in_cam_units]
    bufferOffset = _serializer.float32(obj.current_gain_in_cam_units, buffer, bufferOffset);
    // Serialize message field [min_gain]
    bufferOffset = _serializer.float32(obj.min_gain, buffer, bufferOffset);
    // Serialize message field [max_gain]
    bufferOffset = _serializer.float32(obj.max_gain, buffer, bufferOffset);
    // Serialize message field [current_gain]
    bufferOffset = _serializer.float32(obj.current_gain, buffer, bufferOffset);
    // Serialize message field [min_gamma]
    bufferOffset = _serializer.float32(obj.min_gamma, buffer, bufferOffset);
    // Serialize message field [max_gamma]
    bufferOffset = _serializer.float32(obj.max_gamma, buffer, bufferOffset);
    // Serialize message field [current_gamma]
    bufferOffset = _serializer.float32(obj.current_gamma, buffer, bufferOffset);
    // Serialize message field [brightness_continuous]
    bufferOffset = _serializer.bool(obj.brightness_continuous, buffer, bufferOffset);
    // Serialize message field [gain_auto]
    bufferOffset = _serializer.bool(obj.gain_auto, buffer, bufferOffset);
    // Serialize message field [exposure_auto]
    bufferOffset = _serializer.bool(obj.exposure_auto, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCamPropertiesResponse
    let len;
    let data = new GetCamPropertiesResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_sleeping]
    data.is_sleeping = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [device_user_id]
    data.device_user_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [min_binning_x]
    data.min_binning_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [max_binning_x]
    data.max_binning_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [current_binning_x]
    data.current_binning_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [min_binning_y]
    data.min_binning_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [max_binning_y]
    data.max_binning_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [current_binning_y]
    data.current_binning_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [max_framerate]
    data.max_framerate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_framerate]
    data.current_framerate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min_exposure]
    data.min_exposure = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_exposure]
    data.max_exposure = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_exposure]
    data.current_exposure = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min_gain_in_cam_units]
    data.min_gain_in_cam_units = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_gain_in_cam_units]
    data.max_gain_in_cam_units = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_gain_in_cam_units]
    data.current_gain_in_cam_units = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min_gain]
    data.min_gain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_gain]
    data.max_gain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_gain]
    data.current_gain = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min_gamma]
    data.min_gamma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_gamma]
    data.max_gamma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_gamma]
    data.current_gamma = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [brightness_continuous]
    data.brightness_continuous = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gain_auto]
    data.gain_auto = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [exposure_auto]
    data.exposure_auto = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.device_user_id);
    return length + 89;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/GetCamPropertiesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '14bd64331efb0f665787f525f453c05d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    bool is_sleeping
    
    string device_user_id
    
    int32 min_binning_x
    int32 max_binning_x
    int32 current_binning_x
    
    int32 min_binning_y
    int32 max_binning_y
    int32 current_binning_y
    
    float32 max_framerate
    float32 current_framerate
    
    float32 min_exposure
    float32 max_exposure
    float32 current_exposure
    
    float32 min_gain_in_cam_units
    float32 max_gain_in_cam_units
    float32 current_gain_in_cam_units
    
    float32 min_gain
    float32 max_gain
    float32 current_gain
    
    float32 min_gamma
    float32 max_gamma
    float32 current_gamma
    
    bool brightness_continuous
    bool gain_auto
    bool exposure_auto
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCamPropertiesResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.is_sleeping !== undefined) {
      resolved.is_sleeping = msg.is_sleeping;
    }
    else {
      resolved.is_sleeping = false
    }

    if (msg.device_user_id !== undefined) {
      resolved.device_user_id = msg.device_user_id;
    }
    else {
      resolved.device_user_id = ''
    }

    if (msg.min_binning_x !== undefined) {
      resolved.min_binning_x = msg.min_binning_x;
    }
    else {
      resolved.min_binning_x = 0
    }

    if (msg.max_binning_x !== undefined) {
      resolved.max_binning_x = msg.max_binning_x;
    }
    else {
      resolved.max_binning_x = 0
    }

    if (msg.current_binning_x !== undefined) {
      resolved.current_binning_x = msg.current_binning_x;
    }
    else {
      resolved.current_binning_x = 0
    }

    if (msg.min_binning_y !== undefined) {
      resolved.min_binning_y = msg.min_binning_y;
    }
    else {
      resolved.min_binning_y = 0
    }

    if (msg.max_binning_y !== undefined) {
      resolved.max_binning_y = msg.max_binning_y;
    }
    else {
      resolved.max_binning_y = 0
    }

    if (msg.current_binning_y !== undefined) {
      resolved.current_binning_y = msg.current_binning_y;
    }
    else {
      resolved.current_binning_y = 0
    }

    if (msg.max_framerate !== undefined) {
      resolved.max_framerate = msg.max_framerate;
    }
    else {
      resolved.max_framerate = 0.0
    }

    if (msg.current_framerate !== undefined) {
      resolved.current_framerate = msg.current_framerate;
    }
    else {
      resolved.current_framerate = 0.0
    }

    if (msg.min_exposure !== undefined) {
      resolved.min_exposure = msg.min_exposure;
    }
    else {
      resolved.min_exposure = 0.0
    }

    if (msg.max_exposure !== undefined) {
      resolved.max_exposure = msg.max_exposure;
    }
    else {
      resolved.max_exposure = 0.0
    }

    if (msg.current_exposure !== undefined) {
      resolved.current_exposure = msg.current_exposure;
    }
    else {
      resolved.current_exposure = 0.0
    }

    if (msg.min_gain_in_cam_units !== undefined) {
      resolved.min_gain_in_cam_units = msg.min_gain_in_cam_units;
    }
    else {
      resolved.min_gain_in_cam_units = 0.0
    }

    if (msg.max_gain_in_cam_units !== undefined) {
      resolved.max_gain_in_cam_units = msg.max_gain_in_cam_units;
    }
    else {
      resolved.max_gain_in_cam_units = 0.0
    }

    if (msg.current_gain_in_cam_units !== undefined) {
      resolved.current_gain_in_cam_units = msg.current_gain_in_cam_units;
    }
    else {
      resolved.current_gain_in_cam_units = 0.0
    }

    if (msg.min_gain !== undefined) {
      resolved.min_gain = msg.min_gain;
    }
    else {
      resolved.min_gain = 0.0
    }

    if (msg.max_gain !== undefined) {
      resolved.max_gain = msg.max_gain;
    }
    else {
      resolved.max_gain = 0.0
    }

    if (msg.current_gain !== undefined) {
      resolved.current_gain = msg.current_gain;
    }
    else {
      resolved.current_gain = 0.0
    }

    if (msg.min_gamma !== undefined) {
      resolved.min_gamma = msg.min_gamma;
    }
    else {
      resolved.min_gamma = 0.0
    }

    if (msg.max_gamma !== undefined) {
      resolved.max_gamma = msg.max_gamma;
    }
    else {
      resolved.max_gamma = 0.0
    }

    if (msg.current_gamma !== undefined) {
      resolved.current_gamma = msg.current_gamma;
    }
    else {
      resolved.current_gamma = 0.0
    }

    if (msg.brightness_continuous !== undefined) {
      resolved.brightness_continuous = msg.brightness_continuous;
    }
    else {
      resolved.brightness_continuous = false
    }

    if (msg.gain_auto !== undefined) {
      resolved.gain_auto = msg.gain_auto;
    }
    else {
      resolved.gain_auto = false
    }

    if (msg.exposure_auto !== undefined) {
      resolved.exposure_auto = msg.exposure_auto;
    }
    else {
      resolved.exposure_auto = false
    }

    return resolved;
    }
};

module.exports = {
  Request: GetCamPropertiesRequest,
  Response: GetCamPropertiesResponse,
  md5sum() { return '14bd64331efb0f665787f525f453c05d'; },
  datatype() { return 'camera_control_msgs/GetCamProperties'; }
};
