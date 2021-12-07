// Auto-generated. Do not edit!

// (in-package detection.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class obstacle_bound {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.top_left_x = null;
      this.top_left_y = null;
      this.bottom_right_x = null;
      this.bottom_right_y = null;
      this.distance = null;
    }
    else {
      if (initObj.hasOwnProperty('top_left_x')) {
        this.top_left_x = initObj.top_left_x
      }
      else {
        this.top_left_x = 0;
      }
      if (initObj.hasOwnProperty('top_left_y')) {
        this.top_left_y = initObj.top_left_y
      }
      else {
        this.top_left_y = 0;
      }
      if (initObj.hasOwnProperty('bottom_right_x')) {
        this.bottom_right_x = initObj.bottom_right_x
      }
      else {
        this.bottom_right_x = 0;
      }
      if (initObj.hasOwnProperty('bottom_right_y')) {
        this.bottom_right_y = initObj.bottom_right_y
      }
      else {
        this.bottom_right_y = 0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type obstacle_bound
    // Serialize message field [top_left_x]
    bufferOffset = _serializer.int16(obj.top_left_x, buffer, bufferOffset);
    // Serialize message field [top_left_y]
    bufferOffset = _serializer.int16(obj.top_left_y, buffer, bufferOffset);
    // Serialize message field [bottom_right_x]
    bufferOffset = _serializer.int16(obj.bottom_right_x, buffer, bufferOffset);
    // Serialize message field [bottom_right_y]
    bufferOffset = _serializer.int16(obj.bottom_right_y, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float32(obj.distance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type obstacle_bound
    let len;
    let data = new obstacle_bound(null);
    // Deserialize message field [top_left_x]
    data.top_left_x = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [top_left_y]
    data.top_left_y = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [bottom_right_x]
    data.bottom_right_x = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [bottom_right_y]
    data.bottom_right_y = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'detection/obstacle_bound';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41b15b11d68bbfaa7472618894c283f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 top_left_x
    int16 top_left_y
    int16 bottom_right_x
    int16 bottom_right_y
    float32 distance
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new obstacle_bound(null);
    if (msg.top_left_x !== undefined) {
      resolved.top_left_x = msg.top_left_x;
    }
    else {
      resolved.top_left_x = 0
    }

    if (msg.top_left_y !== undefined) {
      resolved.top_left_y = msg.top_left_y;
    }
    else {
      resolved.top_left_y = 0
    }

    if (msg.bottom_right_x !== undefined) {
      resolved.bottom_right_x = msg.bottom_right_x;
    }
    else {
      resolved.bottom_right_x = 0
    }

    if (msg.bottom_right_y !== undefined) {
      resolved.bottom_right_y = msg.bottom_right_y;
    }
    else {
      resolved.bottom_right_y = 0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    return resolved;
    }
};

module.exports = obstacle_bound;
