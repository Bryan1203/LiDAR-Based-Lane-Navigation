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

class GrabImagesGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.exposure_given = null;
      this.exposure_times = null;
      this.gain_given = null;
      this.gain_values = null;
      this.gamma_given = null;
      this.gamma_values = null;
      this.brightness_given = null;
      this.brightness_values = null;
      this.exposure_auto = null;
      this.gain_auto = null;
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
    // Serializes a message object of type GrabImagesGoal
    // Serialize message field [exposure_given]
    bufferOffset = _serializer.bool(obj.exposure_given, buffer, bufferOffset);
    // Serialize message field [exposure_times]
    bufferOffset = _arraySerializer.float32(obj.exposure_times, buffer, bufferOffset, null);
    // Serialize message field [gain_given]
    bufferOffset = _serializer.bool(obj.gain_given, buffer, bufferOffset);
    // Serialize message field [gain_values]
    bufferOffset = _arraySerializer.float32(obj.gain_values, buffer, bufferOffset, null);
    // Serialize message field [gamma_given]
    bufferOffset = _serializer.bool(obj.gamma_given, buffer, bufferOffset);
    // Serialize message field [gamma_values]
    bufferOffset = _arraySerializer.float32(obj.gamma_values, buffer, bufferOffset, null);
    // Serialize message field [brightness_given]
    bufferOffset = _serializer.bool(obj.brightness_given, buffer, bufferOffset);
    // Serialize message field [brightness_values]
    bufferOffset = _arraySerializer.float32(obj.brightness_values, buffer, bufferOffset, null);
    // Serialize message field [exposure_auto]
    bufferOffset = _serializer.bool(obj.exposure_auto, buffer, bufferOffset);
    // Serialize message field [gain_auto]
    bufferOffset = _serializer.bool(obj.gain_auto, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GrabImagesGoal
    let len;
    let data = new GrabImagesGoal(null);
    // Deserialize message field [exposure_given]
    data.exposure_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [exposure_times]
    data.exposure_times = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [gain_given]
    data.gain_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gain_values]
    data.gain_values = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [gamma_given]
    data.gamma_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gamma_values]
    data.gamma_values = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [brightness_given]
    data.brightness_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [brightness_values]
    data.brightness_values = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [exposure_auto]
    data.exposure_auto = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gain_auto]
    data.gain_auto = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.exposure_times.length;
    length += 4 * object.gain_values.length;
    length += 4 * object.gamma_values.length;
    length += 4 * object.brightness_values.length;
    return length + 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'camera_control_msgs/GrabImagesGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c62b1b0356df57d5bde902f61c90337a';
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
    
    # Only relevant, if exposure_given is true:
    # The list of target exposure times in microseconds.
    # It is possible to grab only one image as well as several images with
    # different exposure times. This values can be overriden from the brightness
    # search, in case that the flag exposure_fixed is not true.
    float32[] exposure_times
    
    # Flag which indicates if the gain is provided and hence should be set before
    # grabbing
    bool gain_given
    
    # Only relevant, if gain_given is true:
    # The target gain in percent of the maximal value the camera supports.
    # For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
    # called 'device specific units'. This value can be overriden from the
    # brightness search, in case that the gain_fixed flag is set to false.
    float32[] gain_values
    
    # Flag which indicates if the gamma value is provided and hence should be set
    # before grabbing
    bool gamma_given
    
    # Only relevant, if gain_given is true:
    # Gamma correction of pixel intensity.
    # Adjusts the brightness of the pixel values output by the camera's sensor
    # to account for a non-linearity in the human perception of brightness or
    # of the display system (such as CRT).
    float32[] gamma_values
    
    # Flag which indicates if the brightness values are provided and hence should
    # be set before grabbing
    bool brightness_given
    
    # Only relevant, if brightness_given is true:
    # The average intensity values of the images. It depends the exposure time
    # as well as the gain setting.
    float32[] brightness_values
    
    # Only relevant, if brightness_given is true:
    # If the camera should try reach the desired brightness, at least one of the
    # following flags MUST be set. If both are set, the interface will use the
    # profile that tries to keep the gain at minimum to reduce white noise.
    # 'exposure_auto' will adapt the exposure time to reach the brightness, wheras
    # 'gain_auto' does so by adapting the gain. If one of these flags is set to
    # false, the connected property will be kept fix.
    # In most of the cases trying to reach a target brightness only by varying the
    # gain and keeping the exposure time fix is not a good approach, because the
    # exposure range is many times higher than the gain range.
    bool exposure_auto
    bool gain_auto
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GrabImagesGoal(null);
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

module.exports = GrabImagesGoal;
