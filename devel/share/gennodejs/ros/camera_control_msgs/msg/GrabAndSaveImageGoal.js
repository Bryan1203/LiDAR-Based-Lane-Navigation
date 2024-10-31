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

class GrabAndSaveImageGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.exposure_given = null;
      this.exposure_time = null;
      this.gain_given = null;
      this.gain_value = null;
      this.gamma_given = null;
      this.gamma_value = null;
      this.brightness_given = null;
      this.brightness_value = null;
      this.exposure_auto = null;
      this.gain_auto = null;
      this.img_storage_path_and_name = null;
    }
    else {
      if (initObj.hasOwnProperty('exposure_given')) {
        this.exposure_given = initObj.exposure_given
      }
      else {
        this.exposure_given = false;
      }
      if (initObj.hasOwnProperty('exposure_time')) {
        this.exposure_time = initObj.exposure_time
      }
      else {
        this.exposure_time = 0.0;
      }
      if (initObj.hasOwnProperty('gain_given')) {
        this.gain_given = initObj.gain_given
      }
      else {
        this.gain_given = false;
      }
      if (initObj.hasOwnProperty('gain_value')) {
        this.gain_value = initObj.gain_value
      }
      else {
        this.gain_value = 0.0;
      }
      if (initObj.hasOwnProperty('gamma_given')) {
        this.gamma_given = initObj.gamma_given
      }
      else {
        this.gamma_given = false;
      }
      if (initObj.hasOwnProperty('gamma_value')) {
        this.gamma_value = initObj.gamma_value
      }
      else {
        this.gamma_value = 0.0;
      }
      if (initObj.hasOwnProperty('brightness_given')) {
        this.brightness_given = initObj.brightness_given
      }
      else {
        this.brightness_given = false;
      }
      if (initObj.hasOwnProperty('brightness_value')) {
        this.brightness_value = initObj.brightness_value
      }
      else {
        this.brightness_value = 0.0;
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
      if (initObj.hasOwnProperty('img_storage_path_and_name')) {
        this.img_storage_path_and_name = initObj.img_storage_path_and_name
      }
      else {
        this.img_storage_path_and_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GrabAndSaveImageGoal
    // Serialize message field [exposure_given]
    bufferOffset = _serializer.bool(obj.exposure_given, buffer, bufferOffset);
    // Serialize message field [exposure_time]
    bufferOffset = _serializer.float32(obj.exposure_time, buffer, bufferOffset);
    // Serialize message field [gain_given]
    bufferOffset = _serializer.bool(obj.gain_given, buffer, bufferOffset);
    // Serialize message field [gain_value]
    bufferOffset = _serializer.float32(obj.gain_value, buffer, bufferOffset);
    // Serialize message field [gamma_given]
    bufferOffset = _serializer.bool(obj.gamma_given, buffer, bufferOffset);
    // Serialize message field [gamma_value]
    bufferOffset = _serializer.float32(obj.gamma_value, buffer, bufferOffset);
    // Serialize message field [brightness_given]
    bufferOffset = _serializer.bool(obj.brightness_given, buffer, bufferOffset);
    // Serialize message field [brightness_value]
    bufferOffset = _serializer.float32(obj.brightness_value, buffer, bufferOffset);
    // Serialize message field [exposure_auto]
    bufferOffset = _serializer.bool(obj.exposure_auto, buffer, bufferOffset);
    // Serialize message field [gain_auto]
    bufferOffset = _serializer.bool(obj.gain_auto, buffer, bufferOffset);
    // Serialize message field [img_storage_path_and_name]
    bufferOffset = _serializer.string(obj.img_storage_path_and_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GrabAndSaveImageGoal
    let len;
    let data = new GrabAndSaveImageGoal(null);
    // Deserialize message field [exposure_given]
    data.exposure_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [exposure_time]
    data.exposure_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gain_given]
    data.gain_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gain_value]
    data.gain_value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gamma_given]
    data.gamma_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gamma_value]
    data.gamma_value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [brightness_given]
    data.brightness_given = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [brightness_value]
    data.brightness_value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [exposure_auto]
    data.exposure_auto = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gain_auto]
    data.gain_auto = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [img_storage_path_and_name]
    data.img_storage_path_and_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.img_storage_path_and_name);
    return length + 26;
  }

  static datatype() {
    // Returns string type for a message object
    return 'camera_control_msgs/GrabAndSaveImageGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c66e7613728d45278cbdc7c5a89f6e08';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    # The 'GrabAndSaveImageAction' has a similar goal as the 'GrabImagesAction' in
    # case of only grabbing one image. It additionally contains a string describing
    # the full storage path of the image to be captured.
    # It wont provide the grabbed image via the result topic. Instead it only
    # returns a flag indicating the success.
    
    ##########################################
    ################## GOAL ##################
    ##########################################
    
    # Flag which indicates if the exposure time is provided and hence should be
    # set before grabbing
    bool exposure_given
    
    # Only relevant, if exposure_given is true:
    # The target exposure time in microseconds. This values can be overriden from
    # the brightness search, in case that the flag exposure_fixed is not true.
    float32 exposure_time
    
    # Flag which indicates if the gain is provided and hence should be set before
    # grabbing
    bool gain_given
    
    # Only relevant, if gain_given is true:
    # The target gain in percent of the maximal value the camera supports.
    # For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
    # called 'device specific units'. This value can be overriden from the
    # brightness search, in case that the gain_fixed flag is set to false.
    float32 gain_value
    
    # Flag which indicates if the gamma value is provided and hence should be set
    # before grabbing
    bool gamma_given
    
    # Only relevant, if gain_given is true:
    # Gamma correction of pixel intensity.
    # Adjusts the brightness of the pixel values output by the camera's sensor
    # to account for a non-linearity in the human perception of brightness or
    # of the display system (such as CRT).
    float32 gamma_value
    
    # Flag which indicates if the brightness value is provided and hence should
    # be set before grabbing
    bool brightness_given
    
    # Only relevant, if brightness_given is true:
    # The average intensity value of the resulting image. It depends the exposure
    # time as well as the gain setting.
    float32 brightness_value
    
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
    
    # File path and image name (including file-extension) to store the grabbed
    # image
    string img_storage_path_and_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GrabAndSaveImageGoal(null);
    if (msg.exposure_given !== undefined) {
      resolved.exposure_given = msg.exposure_given;
    }
    else {
      resolved.exposure_given = false
    }

    if (msg.exposure_time !== undefined) {
      resolved.exposure_time = msg.exposure_time;
    }
    else {
      resolved.exposure_time = 0.0
    }

    if (msg.gain_given !== undefined) {
      resolved.gain_given = msg.gain_given;
    }
    else {
      resolved.gain_given = false
    }

    if (msg.gain_value !== undefined) {
      resolved.gain_value = msg.gain_value;
    }
    else {
      resolved.gain_value = 0.0
    }

    if (msg.gamma_given !== undefined) {
      resolved.gamma_given = msg.gamma_given;
    }
    else {
      resolved.gamma_given = false
    }

    if (msg.gamma_value !== undefined) {
      resolved.gamma_value = msg.gamma_value;
    }
    else {
      resolved.gamma_value = 0.0
    }

    if (msg.brightness_given !== undefined) {
      resolved.brightness_given = msg.brightness_given;
    }
    else {
      resolved.brightness_given = false
    }

    if (msg.brightness_value !== undefined) {
      resolved.brightness_value = msg.brightness_value;
    }
    else {
      resolved.brightness_value = 0.0
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

    if (msg.img_storage_path_and_name !== undefined) {
      resolved.img_storage_path_and_name = msg.img_storage_path_and_name;
    }
    else {
      resolved.img_storage_path_and_name = ''
    }

    return resolved;
    }
};

module.exports = GrabAndSaveImageGoal;
