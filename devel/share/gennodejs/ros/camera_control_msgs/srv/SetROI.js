// Auto-generated. Do not edit!

// (in-package camera_control_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetROIRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_roi = null;
    }
    else {
      if (initObj.hasOwnProperty('target_roi')) {
        this.target_roi = initObj.target_roi
      }
      else {
        this.target_roi = new sensor_msgs.msg.RegionOfInterest();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetROIRequest
    // Serialize message field [target_roi]
    bufferOffset = sensor_msgs.msg.RegionOfInterest.serialize(obj.target_roi, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetROIRequest
    let len;
    let data = new SetROIRequest(null);
    // Deserialize message field [target_roi]
    data.target_roi = sensor_msgs.msg.RegionOfInterest.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetROIRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cf55ea464b4556def55bfcda0d3eab55';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Select a region of interest to get a cropped image.
    # The region is defined by four parameters
    # roi.width: with of the region
    # roi.height: height of the region
    # roi.x_offset at which pixel a long the x axis (horizontal) does the
    # cropped region start
    # roi.y_offset at which pixel a long the y axis (vertical) does the
    # cropped region start
    # The cropped image will then be Image[y_offset:y_offset+vertical, x_offset:x_offset+horizontal]
    # Notice that x_offset cannot be larger than img.width - roi.width
    # The same for y_offset, not larger than img.height - roi.height
    sensor_msgs/RegionOfInterest target_roi
    
    
    ================================================================================
    MSG: sensor_msgs/RegionOfInterest
    # This message is used to specify a region of interest within an image.
    #
    # When used to specify the ROI setting of the camera when the image was
    # taken, the height and width fields should either match the height and
    # width fields for the associated image; or height = width = 0
    # indicates that the full resolution image was captured.
    
    uint32 x_offset  # Leftmost pixel of the ROI
                     # (0 if the ROI includes the left edge of the image)
    uint32 y_offset  # Topmost pixel of the ROI
                     # (0 if the ROI includes the top edge of the image)
    uint32 height    # Height of ROI
    uint32 width     # Width of ROI
    
    # True if a distinct rectified ROI should be calculated from the "raw"
    # ROI in this message. Typically this should be False if the full image
    # is captured (ROI not used), and True if a subwindow is captured (ROI
    # used).
    bool do_rectify
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetROIRequest(null);
    if (msg.target_roi !== undefined) {
      resolved.target_roi = sensor_msgs.msg.RegionOfInterest.Resolve(msg.target_roi)
    }
    else {
      resolved.target_roi = new sensor_msgs.msg.RegionOfInterest()
    }

    return resolved;
    }
};

class SetROIResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reached_roi = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('reached_roi')) {
        this.reached_roi = initObj.reached_roi
      }
      else {
        this.reached_roi = new sensor_msgs.msg.RegionOfInterest();
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
    // Serializes a message object of type SetROIResponse
    // Serialize message field [reached_roi]
    bufferOffset = sensor_msgs.msg.RegionOfInterest.serialize(obj.reached_roi, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetROIResponse
    let len;
    let data = new SetROIResponse(null);
    // Deserialize message field [reached_roi]
    data.reached_roi = sensor_msgs.msg.RegionOfInterest.deserialize(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 18;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera_control_msgs/SetROIResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '70a557e11203ac25f1a7d115b99d7d9b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Exact match can not always reached
    sensor_msgs/RegionOfInterest reached_roi
    bool success
    
    
    ================================================================================
    MSG: sensor_msgs/RegionOfInterest
    # This message is used to specify a region of interest within an image.
    #
    # When used to specify the ROI setting of the camera when the image was
    # taken, the height and width fields should either match the height and
    # width fields for the associated image; or height = width = 0
    # indicates that the full resolution image was captured.
    
    uint32 x_offset  # Leftmost pixel of the ROI
                     # (0 if the ROI includes the left edge of the image)
    uint32 y_offset  # Topmost pixel of the ROI
                     # (0 if the ROI includes the top edge of the image)
    uint32 height    # Height of ROI
    uint32 width     # Width of ROI
    
    # True if a distinct rectified ROI should be calculated from the "raw"
    # ROI in this message. Typically this should be False if the full image
    # is captured (ROI not used), and True if a subwindow is captured (ROI
    # used).
    bool do_rectify
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetROIResponse(null);
    if (msg.reached_roi !== undefined) {
      resolved.reached_roi = sensor_msgs.msg.RegionOfInterest.Resolve(msg.reached_roi)
    }
    else {
      resolved.reached_roi = new sensor_msgs.msg.RegionOfInterest()
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
  Request: SetROIRequest,
  Response: SetROIResponse,
  md5sum() { return '58d19177e6121352c4bf9021da422622'; },
  datatype() { return 'camera_control_msgs/SetROI'; }
};
