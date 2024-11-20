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
let vision_msgs = _finder('vision_msgs');

//-----------------------------------------------------------

class SpatialDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.results = null;
      this.bbox = null;
      this.position = null;
      this.is_tracking = null;
      this.tracking_id = null;
    }
    else {
      if (initObj.hasOwnProperty('results')) {
        this.results = initObj.results
      }
      else {
        this.results = [];
      }
      if (initObj.hasOwnProperty('bbox')) {
        this.bbox = initObj.bbox
      }
      else {
        this.bbox = new vision_msgs.msg.BoundingBox2D();
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('is_tracking')) {
        this.is_tracking = initObj.is_tracking
      }
      else {
        this.is_tracking = false;
      }
      if (initObj.hasOwnProperty('tracking_id')) {
        this.tracking_id = initObj.tracking_id
      }
      else {
        this.tracking_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SpatialDetection
    // Serialize message field [results]
    // Serialize the length for message field [results]
    bufferOffset = _serializer.uint32(obj.results.length, buffer, bufferOffset);
    obj.results.forEach((val) => {
      bufferOffset = vision_msgs.msg.ObjectHypothesis.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [bbox]
    bufferOffset = vision_msgs.msg.BoundingBox2D.serialize(obj.bbox, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [is_tracking]
    bufferOffset = _serializer.bool(obj.is_tracking, buffer, bufferOffset);
    // Serialize message field [tracking_id]
    bufferOffset = _serializer.string(obj.tracking_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SpatialDetection
    let len;
    let data = new SpatialDetection(null);
    // Deserialize message field [results]
    // Deserialize array length for message field [results]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.results = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.results[i] = vision_msgs.msg.ObjectHypothesis.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [bbox]
    data.bbox = vision_msgs.msg.BoundingBox2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [is_tracking]
    data.is_tracking = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tracking_id]
    data.tracking_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.results.length;
    length += _getByteLength(object.tracking_id);
    return length + 73;
  }

  static datatype() {
    // Returns string type for a message object
    return 'depthai_ros_msgs/SpatialDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '503c7980b555f0fd79e92d14cb9ac446';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # Class probabilities
    vision_msgs/ObjectHypothesis[] results
    
    # 2D bounding box surrounding the object.
    vision_msgs/BoundingBox2D bbox
    
    # Center of the detected object in meters 
    geometry_msgs/Point position
    
    # If true, this message contains object tracking information.
    bool is_tracking
    
    # ID used for consistency across multiple detection messages. This value will
    # likely differ from the id field set in each individual ObjectHypothesis.
    # If you set this field, be sure to also set is_tracking to True.
    string tracking_id
    ================================================================================
    MSG: vision_msgs/ObjectHypothesis
    # An object hypothesis that contains no position information.
    
    # The unique numeric ID of object detected. To get additional information about
    #   this ID, such as its human-readable name, listeners should perform a lookup
    #   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
    int64 id
    
    # The probability or confidence value of the detected object. By convention,
    #   this value should lie in the range [0-1].
    float64 score
    ================================================================================
    MSG: vision_msgs/BoundingBox2D
    # A 2D bounding box that can be rotated about its center.
    # All dimensions are in pixels, but represented using floating-point
    #   values to allow sub-pixel precision. If an exact pixel crop is required
    #   for a rotated bounding box, it can be calculated using Bresenham's line
    #   algorithm.
    
    # The 2D position (in pixels) and orientation of the bounding box center.
    geometry_msgs/Pose2D center
    
    # The size (in pixels) of the bounding box surrounding the object relative
    #   to the pose of its center.
    float64 size_x
    float64 size_y
    
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
    const resolved = new SpatialDetection(null);
    if (msg.results !== undefined) {
      resolved.results = new Array(msg.results.length);
      for (let i = 0; i < resolved.results.length; ++i) {
        resolved.results[i] = vision_msgs.msg.ObjectHypothesis.Resolve(msg.results[i]);
      }
    }
    else {
      resolved.results = []
    }

    if (msg.bbox !== undefined) {
      resolved.bbox = vision_msgs.msg.BoundingBox2D.Resolve(msg.bbox)
    }
    else {
      resolved.bbox = new vision_msgs.msg.BoundingBox2D()
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.is_tracking !== undefined) {
      resolved.is_tracking = msg.is_tracking;
    }
    else {
      resolved.is_tracking = false
    }

    if (msg.tracking_id !== undefined) {
      resolved.tracking_id = msg.tracking_id;
    }
    else {
      resolved.tracking_id = ''
    }

    return resolved;
    }
};

module.exports = SpatialDetection;
