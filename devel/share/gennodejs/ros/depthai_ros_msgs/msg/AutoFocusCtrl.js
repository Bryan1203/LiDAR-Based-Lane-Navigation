// Auto-generated. Do not edit!

// (in-package depthai_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class AutoFocusCtrl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.auto_focus_mode = null;
      this.trigger_auto_focus = null;
    }
    else {
      if (initObj.hasOwnProperty('auto_focus_mode')) {
        this.auto_focus_mode = initObj.auto_focus_mode
      }
      else {
        this.auto_focus_mode = 0;
      }
      if (initObj.hasOwnProperty('trigger_auto_focus')) {
        this.trigger_auto_focus = initObj.trigger_auto_focus
      }
      else {
        this.trigger_auto_focus = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AutoFocusCtrl
    // Serialize message field [auto_focus_mode]
    bufferOffset = _serializer.uint8(obj.auto_focus_mode, buffer, bufferOffset);
    // Serialize message field [trigger_auto_focus]
    bufferOffset = _serializer.bool(obj.trigger_auto_focus, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AutoFocusCtrl
    let len;
    let data = new AutoFocusCtrl(null);
    // Deserialize message field [auto_focus_mode]
    data.auto_focus_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [trigger_auto_focus]
    data.trigger_auto_focus = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'depthai_ros_msgs/AutoFocusCtrl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '57bd33867f64daa701fce3a114856f8f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 AF_MODE_AUTO = 0
    uint8 AF_MODE_MACRO = 1
    uint8 AF_MODE_CONTINUOUS_VIDEO = 2
    uint8 AF_MODE_CONTINUOUS_PICTURE = 3
    uint8 AF_MODE_EDOF = 4
    
    uint8 auto_focus_mode
    
    bool trigger_auto_focus
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AutoFocusCtrl(null);
    if (msg.auto_focus_mode !== undefined) {
      resolved.auto_focus_mode = msg.auto_focus_mode;
    }
    else {
      resolved.auto_focus_mode = 0
    }

    if (msg.trigger_auto_focus !== undefined) {
      resolved.trigger_auto_focus = msg.trigger_auto_focus;
    }
    else {
      resolved.trigger_auto_focus = false
    }

    return resolved;
    }
};

// Constants for message
AutoFocusCtrl.Constants = {
  AF_MODE_AUTO: 0,
  AF_MODE_MACRO: 1,
  AF_MODE_CONTINUOUS_VIDEO: 2,
  AF_MODE_CONTINUOUS_PICTURE: 3,
  AF_MODE_EDOF: 4,
}

module.exports = AutoFocusCtrl;
