// Auto-generated. Do not edit!

// (in-package depthai_ros_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class NormalizedImageCropRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.top_left = null;
      this.bottom_right = null;
    }
    else {
      if (initObj.hasOwnProperty('top_left')) {
        this.top_left = initObj.top_left
      }
      else {
        this.top_left = new geometry_msgs.msg.Pose2D();
      }
      if (initObj.hasOwnProperty('bottom_right')) {
        this.bottom_right = initObj.bottom_right
      }
      else {
        this.bottom_right = new geometry_msgs.msg.Pose2D();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NormalizedImageCropRequest
    // Serialize message field [top_left]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.top_left, buffer, bufferOffset);
    // Serialize message field [bottom_right]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.bottom_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NormalizedImageCropRequest
    let len;
    let data = new NormalizedImageCropRequest(null);
    // Deserialize message field [top_left]
    data.top_left = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [bottom_right]
    data.bottom_right = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'depthai_ros_msgs/NormalizedImageCropRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '59bb83618d002416c9960e7cc0740912';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose2D top_left
    geometry_msgs/Pose2D bottom_right
    
    ================================================================================
    MSG: geometry_msgs/Pose2D
    # Deprecated
    # Please use the full 3D pose.
    
    # In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.
    
    # If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.
    
    
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NormalizedImageCropRequest(null);
    if (msg.top_left !== undefined) {
      resolved.top_left = geometry_msgs.msg.Pose2D.Resolve(msg.top_left)
    }
    else {
      resolved.top_left = new geometry_msgs.msg.Pose2D()
    }

    if (msg.bottom_right !== undefined) {
      resolved.bottom_right = geometry_msgs.msg.Pose2D.Resolve(msg.bottom_right)
    }
    else {
      resolved.bottom_right = new geometry_msgs.msg.Pose2D()
    }

    return resolved;
    }
};

class NormalizedImageCropResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NormalizedImageCropResponse
    // Serialize message field [status]
    bufferOffset = _serializer.int64(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NormalizedImageCropResponse
    let len;
    let data = new NormalizedImageCropResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'depthai_ros_msgs/NormalizedImageCropResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4107476a6271fc2684d94be17d33f854';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NormalizedImageCropResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: NormalizedImageCropRequest,
  Response: NormalizedImageCropResponse,
  md5sum() { return '35d4b74aa140d5d6ce7c27c43314e948'; },
  datatype() { return 'depthai_ros_msgs/NormalizedImageCrop'; }
};
