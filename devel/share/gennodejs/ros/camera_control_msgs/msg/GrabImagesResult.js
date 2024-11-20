// Auto-generated. Do not edit!

// (in-package camera_control_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class GrabImagesResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.images = null;
      this.cam_info = null;
      this.reached_exposure_times = null;
      this.reached_brightness_values = null;
      this.reached_gain_values = null;
      this.reached_gamma_values = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('images')) {
        this.images = initObj.images
      }
      else {
        this.images = [];
      }
      if (initObj.hasOwnProperty('cam_info')) {
        this.cam_info = initObj.cam_info
      }
      else {
        this.cam_info = new sensor_msgs.msg.CameraInfo();
      }
      if (initObj.hasOwnProperty('reached_exposure_times')) {
        this.reached_exposure_times = initObj.reached_exposure_times
      }
      else {
        this.reached_exposure_times = [];
      }
      if (initObj.hasOwnProperty('reached_brightness_values')) {
        this.reached_brightness_values = initObj.reached_brightness_values
      }
      else {
        this.reached_brightness_values = [];
      }
      if (initObj.hasOwnProperty('reached_gain_values')) {
        this.reached_gain_values = initObj.reached_gain_values
      }
      else {
        this.reached_gain_values = [];
      }
      if (initObj.hasOwnProperty('reached_gamma_values')) {
        this.reached_gamma_values = initObj.reached_gamma_values
      }
      else {
        this.reached_gamma_values = [];
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
    // Serializes a message object of type GrabImagesResult
    // Serialize message field [images]
    // Serialize the length for message field [images]
    bufferOffset = _serializer.uint32(obj.images.length, buffer, bufferOffset);
    obj.images.forEach((val) => {
      bufferOffset = sensor_msgs.msg.Image.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [cam_info]
    bufferOffset = sensor_msgs.msg.CameraInfo.serialize(obj.cam_info, buffer, bufferOffset);
    // Serialize message field [reached_exposure_times]
    bufferOffset = _arraySerializer.float32(obj.reached_exposure_times, buffer, bufferOffset, null);
    // Serialize message field [reached_brightness_values]
    bufferOffset = _arraySerializer.float32(obj.reached_brightness_values, buffer, bufferOffset, null);
    // Serialize message field [reached_gain_values]
    bufferOffset = _arraySerializer.float32(obj.reached_gain_values, buffer, bufferOffset, null);
    // Serialize message field [reached_gamma_values]
    bufferOffset = _arraySerializer.float32(obj.reached_gamma_values, buffer, bufferOffset, null);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GrabImagesResult
    let len;
    let data = new GrabImagesResult(null);
    // Deserialize message field [images]
    // Deserialize array length for message field [images]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.images = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.images[i] = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [cam_info]
    data.cam_info = sensor_msgs.msg.CameraInfo.deserialize(buffer, bufferOffset);
    // Deserialize message field [reached_exposure_times]
    data.reached_exposure_times = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [reached_brightness_values]
    data.reached_brightness_values = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [reached_gain_values]
    data.reached_gain_values = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [reached_gamma_values]
    data.reached_gamma_values = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.images.forEach((val) => {
      length += sensor_msgs.msg.Image.getMessageSize(val);
    });
    length += sensor_msgs.msg.CameraInfo.getMessageSize(object.cam_info);
    length += 4 * object.reached_exposure_times.length;
    length += 4 * object.reached_brightness_values.length;
    length += 4 * object.reached_gain_values.length;
    length += 4 * object.reached_gamma_values.length;
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'camera_control_msgs/GrabImagesResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f2d7966fcbce7df3d70e766fac52078d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    ##########################################
    ################# RESULT #################
    ##########################################
    
    # The resulting images with the inquired image intensity settings.
    # The size of the vector equals the size of the exposure_times or the
    # brightness_values-vector
    sensor_msgs/Image[] images
    
    # The CameraInfo obejct describing the camera properties for the above image
    # sequence. Static in many cases, but can also support variable binning setting
    sensor_msgs/CameraInfo cam_info
    
    # The reached values of the images e.g. the values that were set to the camera
    # before the grab
    float32[] reached_exposure_times
    
    float32[] reached_brightness_values
    
    float32[] reached_gain_values
    
    float32[] reached_gamma_values
    
    # Flag which indicates the success of the grabbing action
    # In case of failure, the images-vector contains only the images, that could be
    # grabbed before the failure occurred.
    bool success
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
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
    MSG: sensor_msgs/CameraInfo
    # This message defines meta information for a camera. It should be in a
    # camera namespace on topic "camera_info" and accompanied by up to five
    # image topics named:
    #
    #   image_raw - raw data from the camera driver, possibly Bayer encoded
    #   image            - monochrome, distorted
    #   image_color      - color, distorted
    #   image_rect       - monochrome, rectified
    #   image_rect_color - color, rectified
    #
    # The image_pipeline contains packages (image_proc, stereo_image_proc)
    # for producing the four processed image topics from image_raw and
    # camera_info. The meaning of the camera parameters are described in
    # detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
    #
    # The image_geometry package provides a user-friendly interface to
    # common operations using this meta information. If you want to, e.g.,
    # project a 3d point into image coordinates, we strongly recommend
    # using image_geometry.
    #
    # If the camera is uncalibrated, the matrices D, K, R, P should be left
    # zeroed out. In particular, clients may assume that K[0] == 0.0
    # indicates an uncalibrated camera.
    
    #######################################################################
    #                     Image acquisition info                          #
    #######################################################################
    
    # Time of image acquisition, camera coordinate frame ID
    Header header    # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into the plane of the image
    
    
    #######################################################################
    #                      Calibration Parameters                         #
    #######################################################################
    # These are fixed during camera calibration. Their values will be the #
    # same in all messages until the camera is recalibrated. Note that    #
    # self-calibrating systems may "recalibrate" frequently.              #
    #                                                                     #
    # The internal parameters can be used to warp a raw (distorted) image #
    # to:                                                                 #
    #   1. An undistorted image (requires D and K)                        #
    #   2. A rectified image (requires D, K, R)                           #
    # The projection matrix P projects 3D points into the rectified image.#
    #######################################################################
    
    # The image dimensions with which the camera was calibrated. Normally
    # this will be the full camera resolution in pixels.
    uint32 height
    uint32 width
    
    # The distortion model used. Supported models are listed in
    # sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
    # simple model of radial and tangential distortion - is sufficient.
    string distortion_model
    
    # The distortion parameters, size depending on the distortion model.
    # For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    float64[] D
    
    # Intrinsic camera matrix for the raw (distorted) images.
    #     [fx  0 cx]
    # K = [ 0 fy cy]
    #     [ 0  0  1]
    # Projects 3D points in the camera coordinate frame to 2D pixel
    # coordinates using the focal lengths (fx, fy) and principal point
    # (cx, cy).
    float64[9]  K # 3x3 row-major matrix
    
    # Rectification matrix (stereo cameras only)
    # A rotation matrix aligning the camera coordinate system to the ideal
    # stereo image plane so that epipolar lines in both stereo images are
    # parallel.
    float64[9]  R # 3x3 row-major matrix
    
    # Projection/camera matrix
    #     [fx'  0  cx' Tx]
    # P = [ 0  fy' cy' Ty]
    #     [ 0   0   1   0]
    # By convention, this matrix specifies the intrinsic (camera) matrix
    #  of the processed (rectified) image. That is, the left 3x3 portion
    #  is the normal camera intrinsic matrix for the rectified image.
    # It projects 3D points in the camera coordinate frame to 2D pixel
    #  coordinates using the focal lengths (fx', fy') and principal point
    #  (cx', cy') - these may differ from the values in K.
    # For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
    #  also have R = the identity and P[1:3,1:3] = K.
    # For a stereo pair, the fourth column [Tx Ty 0]' is related to the
    #  position of the optical center of the second camera in the first
    #  camera's frame. We assume Tz = 0 so both cameras are in the same
    #  stereo image plane. The first camera always has Tx = Ty = 0. For
    #  the right (second) camera of a horizontal stereo pair, Ty = 0 and
    #  Tx = -fx' * B, where B is the baseline between the cameras.
    # Given a 3D point [X Y Z]', the projection (x, y) of the point onto
    #  the rectified image is given by:
    #  [u v w]' = P * [X Y Z 1]'
    #         x = u / w
    #         y = v / w
    #  This holds for both images of a stereo pair.
    float64[12] P # 3x4 row-major matrix
    
    
    #######################################################################
    #                      Operational Parameters                         #
    #######################################################################
    # These define the image region actually captured by the camera       #
    # driver. Although they affect the geometry of the output image, they #
    # may be changed freely without recalibrating the camera.             #
    #######################################################################
    
    # Binning refers here to any camera setting which combines rectangular
    #  neighborhoods of pixels into larger "super-pixels." It reduces the
    #  resolution of the output image to
    #  (width / binning_x) x (height / binning_y).
    # The default values binning_x = binning_y = 0 is considered the same
    #  as binning_x = binning_y = 1 (no subsampling).
    uint32 binning_x
    uint32 binning_y
    
    # Region of interest (subwindow of full camera resolution), given in
    #  full resolution (unbinned) image coordinates. A particular ROI
    #  always denotes the same window of pixels on the camera sensor,
    #  regardless of binning settings.
    # The default setting of roi (all values 0) is considered the same as
    #  full resolution (roi.width = width, roi.height = height).
    RegionOfInterest roi
    
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
    const resolved = new GrabImagesResult(null);
    if (msg.images !== undefined) {
      resolved.images = new Array(msg.images.length);
      for (let i = 0; i < resolved.images.length; ++i) {
        resolved.images[i] = sensor_msgs.msg.Image.Resolve(msg.images[i]);
      }
    }
    else {
      resolved.images = []
    }

    if (msg.cam_info !== undefined) {
      resolved.cam_info = sensor_msgs.msg.CameraInfo.Resolve(msg.cam_info)
    }
    else {
      resolved.cam_info = new sensor_msgs.msg.CameraInfo()
    }

    if (msg.reached_exposure_times !== undefined) {
      resolved.reached_exposure_times = msg.reached_exposure_times;
    }
    else {
      resolved.reached_exposure_times = []
    }

    if (msg.reached_brightness_values !== undefined) {
      resolved.reached_brightness_values = msg.reached_brightness_values;
    }
    else {
      resolved.reached_brightness_values = []
    }

    if (msg.reached_gain_values !== undefined) {
      resolved.reached_gain_values = msg.reached_gain_values;
    }
    else {
      resolved.reached_gain_values = []
    }

    if (msg.reached_gamma_values !== undefined) {
      resolved.reached_gamma_values = msg.reached_gamma_values;
    }
    else {
      resolved.reached_gamma_values = []
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

module.exports = GrabImagesResult;
