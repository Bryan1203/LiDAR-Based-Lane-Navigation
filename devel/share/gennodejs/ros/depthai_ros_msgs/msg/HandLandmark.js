// Auto-generated. Do not edit!

// (in-package depthai_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class HandLandmark {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.label = null;
      this.lm_score = null;
      this.landmark = null;
      this.position = null;
      this.is_spatial = null;
    }
    else {
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = '';
      }
      if (initObj.hasOwnProperty('lm_score')) {
        this.lm_score = initObj.lm_score
      }
      else {
        this.lm_score = 0.0;
      }
      if (initObj.hasOwnProperty('landmark')) {
        this.landmark = initObj.landmark
      }
      else {
        this.landmark = [];
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('is_spatial')) {
        this.is_spatial = initObj.is_spatial
      }
      else {
        this.is_spatial = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HandLandmark
    // Serialize message field [label]
    bufferOffset = _serializer.string(obj.label, buffer, bufferOffset);
    // Serialize message field [lm_score]
    bufferOffset = _serializer.float32(obj.lm_score, buffer, bufferOffset);
    // Serialize message field [landmark]
    // Serialize the length for message field [landmark]
    bufferOffset = _serializer.uint32(obj.landmark.length, buffer, bufferOffset);
    obj.landmark.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose2D.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [is_spatial]
    bufferOffset = _serializer.bool(obj.is_spatial, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HandLandmark
    let len;
    let data = new HandLandmark(null);
    // Deserialize message field [label]
    data.label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lm_score]
    data.lm_score = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [landmark]
    // Deserialize array length for message field [landmark]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.landmark = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.landmark[i] = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [is_spatial]
    data.is_spatial = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.label);
    length += 24 * object.landmark.length;
    return length + 37;
  }

  static datatype() {
    // Returns string type for a message object
    return 'depthai_ros_msgs/HandLandmark';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e126f09ad65617304665e31f1a5736de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # Center of the 
    
    string label
    
    # Landmarks score.
    float32 lm_score
    
    geometry_msgs/Pose2D[] landmark
    
    geometry_msgs/Point position
    bool is_spatial
    
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
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HandLandmark(null);
    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = ''
    }

    if (msg.lm_score !== undefined) {
      resolved.lm_score = msg.lm_score;
    }
    else {
      resolved.lm_score = 0.0
    }

    if (msg.landmark !== undefined) {
      resolved.landmark = new Array(msg.landmark.length);
      for (let i = 0; i < resolved.landmark.length; ++i) {
        resolved.landmark[i] = geometry_msgs.msg.Pose2D.Resolve(msg.landmark[i]);
      }
    }
    else {
      resolved.landmark = []
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.is_spatial !== undefined) {
      resolved.is_spatial = msg.is_spatial;
    }
    else {
      resolved.is_spatial = false
    }

    return resolved;
    }
};

module.exports = HandLandmark;
