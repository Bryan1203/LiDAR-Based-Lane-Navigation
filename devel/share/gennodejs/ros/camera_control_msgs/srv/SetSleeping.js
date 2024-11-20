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

class SetSleepingRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.set_sleeping = null;
    }
    else {
      if (initObj.hasOwnProperty('set_sleeping')) {
        this.set_sleeping = initObj.set_sleeping
      }
      else {
        this.set_sleeping = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetSleepingRequest
    // Serialize message field [set_sleeping]
    bufferOffset = _serializer.bool(obj.set_sleeping, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSleepingRequest
    let len;
    let data = new SetSleepingRequest(null);
    // Deserialize message field [set_sleeping]
    data.set_sleeping = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetSleepingRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '58aad3b532b8c4776fd592aec6441836';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # If the camera runs in topic mode (continuously publishing images over the
    # topics respecting the desired frame_rate) this service offers the posibillity
    # to pause the image acquisition. To restart the grabbing, this service should
    # be called again with set_sleeping set to false
    bool set_sleeping
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetSleepingRequest(null);
    if (msg.set_sleeping !== undefined) {
      resolved.set_sleeping = msg.set_sleeping;
    }
    else {
      resolved.set_sleeping = false
    }

    return resolved;
    }
};

class SetSleepingResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetSleepingResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSleepingResponse
    let len;
    let data = new SetSleepingResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetSleepingResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetSleepingResponse(null);
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
  Request: SetSleepingRequest,
  Response: SetSleepingResponse,
  md5sum() { return 'f51754709312645db79ada18bc0254a3'; },
  datatype() { return 'camera_control_msgs/SetSleeping'; }
};
