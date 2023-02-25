// Auto-generated. Do not edit!

// (in-package courseworks_answer_w1.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class signal_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time_x = null;
      this.signal_y = null;
    }
    else {
      if (initObj.hasOwnProperty('time_x')) {
        this.time_x = initObj.time_x
      }
      else {
        this.time_x = 0.0;
      }
      if (initObj.hasOwnProperty('signal_y')) {
        this.signal_y = initObj.signal_y
      }
      else {
        this.signal_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type signal_msg
    // Serialize message field [time_x]
    bufferOffset = _serializer.float32(obj.time_x, buffer, bufferOffset);
    // Serialize message field [signal_y]
    bufferOffset = _serializer.float32(obj.signal_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type signal_msg
    let len;
    let data = new signal_msg(null);
    // Deserialize message field [time_x]
    data.time_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [signal_y]
    data.signal_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'courseworks_answer_w1/signal_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7a11f0834f5ad70ec78337929f76544a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 time_x
    float32 signal_y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new signal_msg(null);
    if (msg.time_x !== undefined) {
      resolved.time_x = msg.time_x;
    }
    else {
      resolved.time_x = 0.0
    }

    if (msg.signal_y !== undefined) {
      resolved.signal_y = msg.signal_y;
    }
    else {
      resolved.signal_y = 0.0
    }

    return resolved;
    }
};

module.exports = signal_msg;
