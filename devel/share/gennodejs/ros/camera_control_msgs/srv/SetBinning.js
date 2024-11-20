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

class SetBinningRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_binning_x = null;
      this.target_binning_y = null;
    }
    else {
      if (initObj.hasOwnProperty('target_binning_x')) {
        this.target_binning_x = initObj.target_binning_x
      }
      else {
        this.target_binning_x = 0;
      }
      if (initObj.hasOwnProperty('target_binning_y')) {
        this.target_binning_y = initObj.target_binning_y
      }
      else {
        this.target_binning_y = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetBinningRequest
    // Serialize message field [target_binning_x]
    bufferOffset = _serializer.uint32(obj.target_binning_x, buffer, bufferOffset);
    // Serialize message field [target_binning_y]
    bufferOffset = _serializer.uint32(obj.target_binning_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetBinningRequest
    let len;
    let data = new SetBinningRequest(null);
    // Deserialize message field [target_binning_x]
    data.target_binning_x = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [target_binning_y]
    data.target_binning_y = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetBinningRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '070d3ce2d671acce3b96edb95ae60995';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Binning factor to get downsampled images. It refers here to any camera
    # setting which combines rectangular neighborhoods of pixels into larger
    # "super-pixels." It reduces the resolution of the output image to
    # (width / binning_x) x (height / binning_y).
    # The default values binning_x = binning_y = 0 are considered the same
    # as binning_x = binning_y = 1 (no subsampling).
    # Calling this service with target binning values will change the binning entry
    # in the published camera_info_msg
    uint32 target_binning_x
    uint32 target_binning_y
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetBinningRequest(null);
    if (msg.target_binning_x !== undefined) {
      resolved.target_binning_x = msg.target_binning_x;
    }
    else {
      resolved.target_binning_x = 0
    }

    if (msg.target_binning_y !== undefined) {
      resolved.target_binning_y = msg.target_binning_y;
    }
    else {
      resolved.target_binning_y = 0
    }

    return resolved;
    }
};

class SetBinningResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reached_binning_x = null;
      this.reached_binning_y = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('reached_binning_x')) {
        this.reached_binning_x = initObj.reached_binning_x
      }
      else {
        this.reached_binning_x = 0;
      }
      if (initObj.hasOwnProperty('reached_binning_y')) {
        this.reached_binning_y = initObj.reached_binning_y
      }
      else {
        this.reached_binning_y = 0;
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
    // Serializes a message object of type SetBinningResponse
    // Serialize message field [reached_binning_x]
    bufferOffset = _serializer.uint32(obj.reached_binning_x, buffer, bufferOffset);
    // Serialize message field [reached_binning_y]
    bufferOffset = _serializer.uint32(obj.reached_binning_y, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetBinningResponse
    let len;
    let data = new SetBinningResponse(null);
    // Deserialize message field [reached_binning_x]
    data.reached_binning_x = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [reached_binning_y]
    data.reached_binning_y = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetBinningResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '519ec992dd33973d0740b5940161bf8e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Exact match can not always be reached
    uint32 reached_binning_x
    uint32 reached_binning_y
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetBinningResponse(null);
    if (msg.reached_binning_x !== undefined) {
      resolved.reached_binning_x = msg.reached_binning_x;
    }
    else {
      resolved.reached_binning_x = 0
    }

    if (msg.reached_binning_y !== undefined) {
      resolved.reached_binning_y = msg.reached_binning_y;
    }
    else {
      resolved.reached_binning_y = 0
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
  Request: SetBinningRequest,
  Response: SetBinningResponse,
  md5sum() { return 'fb7b82cb93bc5513ade2d601c7a7d975'; },
  datatype() { return 'camera_control_msgs/SetBinning'; }
};
