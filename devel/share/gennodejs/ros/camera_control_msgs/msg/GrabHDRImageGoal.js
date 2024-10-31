// Auto-generated. Do not edit!

// (in-package camera_control_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class GrabHDRImageGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.exposure_given = null;
      this.exposure_times = null;
      this.exposure_fixed = null;
      this.brightness_given = null;
      this.brightness_values = null;
      this.brightness_continuous = null;
      this.gain_given = null;
      this.gain_values = null;
      this.gain_fixed = null;
      this.gamma_given = null;
      this.gamma_values = null;
    }
    else {
      if (initObj.hasOwnProperty('exposure_given')) {
        this.exposure_given = initObj.exposure_given
      }
      else {
        this.exposure_given = false;
      }
      if (initObj.hasOwnProperty('exposure_times')) {
        this.exposure_times = initObj.exposure_times
      }
      else {
        this.exposure_times = [];
      }
      if (initObj.hasOwnProperty('exposure_fixed')) {
        this.exposure_fixed = initObj.exposure_fixed
      }
      else {
        this.exposure_fixed = false;
      }
      if (initObj.hasOwnProperty('brightness_given')) {
        this.brightness_given = initObj.brightness_given
      }
      else {
        this.brightness_given = false;
      }
      if (initObj.hasOwnProperty('brightness_values')) {
        this.brightness_values = initObj.brightness_values
      }
      else {
        this.brightness_values = [];
      }
      if (initObj.hasOwnProperty('brightness_continuous')) {
        this.brightness_continuous = initObj.brightness_continuous
      }
      else {
        this.brightness_continuous = false;
      }
      if (initObj.hasOwnProperty('gain_given')) {
        this.gain_given = initObj.gain_given
      }
      else {
        this.gain_given = false;
      }
      if (initObj.hasOwnProperty('gain_values')) {
        this.gain_values = initObj.gain_values
      }
      else {
        this.gain_values = [];
      }
      if (initObj.hasOwnProperty('gain_fixed')) {
        this.gain_fixed = initObj.gain_fixed
      }
      else {
        this.gain_fixed = false;
      }
      if (initObj.hasOwnProperty('gamma_given')) {
        this.gamma_given = initObj.gamma_given
      }
      else {
        this.gamma_given = false;
      }
      if (initObj.hasOwnProperty('gamma_values')) {
        this.gamma_values = initObj.gamma_values
      }
      else {
        this.gamma_values = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GrabHDRImageGoal
    // Serialize message field [exposure_given]
    bufferOffset = _serializer.bool(obj.exposure_given, buffer, bufferOffset);
    // Serialize message field [exposure_times]
    bufferOffset = _arraySerializer.float32(obj.exposure_times, buffer, bufferOffset, null);
    // Serialize message field [exposure_fixed]
    bufferOffset = _serializer.bool(obj.exposure_fixed, buffer, bufferOffset);
    // Serialize message field [brightness_given]
    bufferOffset = _serializer.bool(obj.brightness_given, buffer, bufferOffset);
    // Serialize message field [brightness_values]
    bufferOffset = _arraySerializer.float32(obj.brightness_values, buffer, bufferOffset, null);
    // Serialize message field [brightness_continuous]
    bufferOffset = _serializer.bool(obj.brightness_continuous, buffer, bufferOffset);
    // Serialize message field [gain_given]
    bufferOffset = _serializer.bool(obj.gain_given, buffer, bufferOffset);
    // Serialize message field [gain_values]
    bufferOffset = _arraySerializer.float32(obj.gain_values, buffer, bufferOffset, null);
    // Serialize message field [gain_fixed]
    bufferOffset = _serializer.bool(obj.gain_fixed, buffer, bufferOffset);
    // Serialize message field [gamma_given]
    bufferOffset = _serializer.bool(obj.gamma_given, buffer, bufferOffset);
    // Serialize message field [gamma_values]
    bufferOffset = _arraySerializer.float32(obj.gamma_values, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GrabHDRImageGoal
    let len;
    let data = new GrabHDRImageGoal(null);
    // Deserialize message field [exposure_given]
    data.exposure_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [exposure_times]
    data.exposure_times = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [exposure_fixed]
    data.exposure_fixed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [brightness_given]
    data.brightness_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [brightness_values]
    data.brightness_values = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [brightness_continuous]
    data.brightness_continuous = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gain_given]
    data.gain_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gain_values]
    data.gain_values = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [gain_fixed]
    data.gain_fixed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gamma_given]
    data.gamma_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gamma_values]
    data.gamma_values = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.exposure_times.length;
    length += 4 * object.brightness_values.length;
    length += 4 * object.gain_values.length;
    length += 4 * object.gamma_values.length;
    return length + 23;
  }

  static datatype() {
    // Returns string type for a message object
    return 'camera_control_msgs/GrabHDRImageGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e5897a9b679b570ea1e6aa1d1ce9869';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    ##########################################
    ################## GOAL ##################
    ##########################################
    
    # Flag which indicates if the exposure times are provided and hence should be
    # set before grabbing
    bool exposure_given
    
    # The list of target exposure time in microseconds.
    # It is possible to grab only one image as well as several images with
    # different exposure times. This values can be overriden from the brightness
    # search, in case that the flag exposure_fixed is not true.
    # The interface will only try to set these values if the exposure_given flag is
    # set to true.
    float32[] exposure_times
    
    # If the exposure_fixed flag is set, the exposure time will stay fix in
    # case of a brightness search. Hence the target brightness will be reached
    # only by varying the gain
    bool exposure_fixed
    
    
    # Flag which indicates if the brightness values are provided and hence should
    # be set before grabbing
    bool brightness_given
    
    # The average intensity values of the images. It depends the exposure time
    # as well as the gain setting. The interface will only try to reach this
    # value if the brightness_given flag is set to true.
    float32[] brightness_values
    
    # The brightness_continuous flag controls the auto brightness function.
    # If it is set to false, the given brightness will only be reached once.
    # Hence changing light conditions lead to changing brightness values.
    # If it is set to true, the given brightness will be reached continuously,
    # trying to adapt to changing light conditions.
    bool brightness_continuous
    
    
    # Flag which indicates if the gain is provided and hence should be set before
    # grabbing
    bool gain_given
    
    # The target gain values in percent of the maximal value the camera supports
    # For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
    # called 'device specific units'. This value can be overriden from the
    # brightness search, in case that the gain_fixed flag is set to false.
    # The interface will only try to reach this values if the gain_given flag is
    # set to true.
    float32[] gain_values
    
    # If the gain_fixed flag is set, the gain value will stay fix in
    # case of a brightness search. Hence the target brightness will be reached
    # only by varying the exposure
    bool gain_fixed
    
    # Flag which indicates if the gamma value is provided and hence should be set
    # before grabbing
    bool gamma_given
    
    # Gamma correction of pixel intensity.
    # Adjusts the brightness of the pixel values output by the camera's sensor
    # to account for a non-linearity in the human perception of brightness or
    # of the display system (such as CRT).
    # The interface will only try to reach this value if the gamma_given flag is
    # set to true.
    float32[] gamma_values
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GrabHDRImageGoal(null);
    if (msg.exposure_given !== undefined) {
      resolved.exposure_given = msg.exposure_given;
    }
    else {
      resolved.exposure_given = false
    }

    if (msg.exposure_times !== undefined) {
      resolved.exposure_times = msg.exposure_times;
    }
    else {
      resolved.exposure_times = []
    }

    if (msg.exposure_fixed !== undefined) {
      resolved.exposure_fixed = msg.exposure_fixed;
    }
    else {
      resolved.exposure_fixed = false
    }

    if (msg.brightness_given !== undefined) {
      resolved.brightness_given = msg.brightness_given;
    }
    else {
      resolved.brightness_given = false
    }

    if (msg.brightness_values !== undefined) {
      resolved.brightness_values = msg.brightness_values;
    }
    else {
      resolved.brightness_values = []
    }

    if (msg.brightness_continuous !== undefined) {
      resolved.brightness_continuous = msg.brightness_continuous;
    }
    else {
      resolved.brightness_continuous = false
    }

    if (msg.gain_given !== undefined) {
      resolved.gain_given = msg.gain_given;
    }
    else {
      resolved.gain_given = false
    }

    if (msg.gain_values !== undefined) {
      resolved.gain_values = msg.gain_values;
    }
    else {
      resolved.gain_values = []
    }

    if (msg.gain_fixed !== undefined) {
      resolved.gain_fixed = msg.gain_fixed;
    }
    else {
      resolved.gain_fixed = false
    }

    if (msg.gamma_given !== undefined) {
      resolved.gamma_given = msg.gamma_given;
    }
    else {
      resolved.gamma_given = false
    }

    if (msg.gamma_values !== undefined) {
      resolved.gamma_values = msg.gamma_values;
    }
    else {
      resolved.gamma_values = []
    }

    return resolved;
    }
};

module.exports = GrabHDRImageGoal;
