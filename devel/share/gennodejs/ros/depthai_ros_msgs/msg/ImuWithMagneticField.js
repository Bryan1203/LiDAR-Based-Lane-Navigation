// Auto-generated. Do not edit!

// (in-package depthai_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class ImuWithMagneticField {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.imu = null;
      this.field = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('imu')) {
        this.imu = initObj.imu
      }
      else {
        this.imu = new sensor_msgs.msg.Imu();
      }
      if (initObj.hasOwnProperty('field')) {
        this.field = initObj.field
      }
      else {
        this.field = new sensor_msgs.msg.MagneticField();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImuWithMagneticField
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [imu]
    bufferOffset = sensor_msgs.msg.Imu.serialize(obj.imu, buffer, bufferOffset);
    // Serialize message field [field]
    bufferOffset = sensor_msgs.msg.MagneticField.serialize(obj.field, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImuWithMagneticField
    let len;
    let data = new ImuWithMagneticField(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [imu]
    data.imu = sensor_msgs.msg.Imu.deserialize(buffer, bufferOffset);
    // Deserialize message field [field]
    data.field = sensor_msgs.msg.MagneticField.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += sensor_msgs.msg.Imu.getMessageSize(object.imu);
    length += sensor_msgs.msg.MagneticField.getMessageSize(object.field);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'depthai_ros_msgs/ImuWithMagneticField';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7542bff2bd62a394f0408d003d2b52df';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    sensor_msgs/Imu imu
    sensor_msgs/MagneticField field
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: sensor_msgs/Imu
    # This is a message to hold data from an IMU (Inertial Measurement Unit)
    #
    # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    #
    # If the covariance of the measurement is known, it should be filled in (if all you know is the 
    # variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
    # A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
    # data a covariance will have to be assumed or gotten from some other source
    #
    # If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
    # estimate), please set element 0 of the associated covariance matrix to -1
    # If you are interpreting this message, please check for a value of -1 in the first element of each 
    # covariance matrix, and disregard the associated estimate.
    
    Header header
    
    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance # Row major x, y z 
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: sensor_msgs/MagneticField
     # Measurement of the Magnetic Field vector at a specific location.
    
     # If the covariance of the measurement is known, it should be filled in
     # (if all you know is the variance of each measurement, e.g. from the datasheet,
     #just put those along the diagonal)
     # A covariance matrix of all zeros will be interpreted as "covariance unknown",
     # and to use the data a covariance will have to be assumed or gotten from some
     # other source
    
    
     Header header                        # timestamp is the time the
                                          # field was measured
                                          # frame_id is the location and orientation
                                          # of the field measurement
    
     geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
                                          # field vector in Tesla
                                          # If your sensor does not output 3 axes,
                                          # put NaNs in the components not reported.
    
     float64[9] magnetic_field_covariance # Row major about x, y, z axes
                                          # 0 is interpreted as variance unknown
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ImuWithMagneticField(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.imu !== undefined) {
      resolved.imu = sensor_msgs.msg.Imu.Resolve(msg.imu)
    }
    else {
      resolved.imu = new sensor_msgs.msg.Imu()
    }

    if (msg.field !== undefined) {
      resolved.field = sensor_msgs.msg.MagneticField.Resolve(msg.field)
    }
    else {
      resolved.field = new sensor_msgs.msg.MagneticField()
    }

    return resolved;
    }
};

module.exports = ImuWithMagneticField;
