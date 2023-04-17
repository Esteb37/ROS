// Auto-generated. Do not edit!

// (in-package open_path_follower.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class script_select {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.script_select = null;
      this.type_select = null;
      this.turn_velocity = null;
      this.move_velocity = null;
      this.total_time = null;
      this.square_length = null;
    }
    else {
      if (initObj.hasOwnProperty('script_select')) {
        this.script_select = initObj.script_select
      }
      else {
        this.script_select = '';
      }
      if (initObj.hasOwnProperty('type_select')) {
        this.type_select = initObj.type_select
      }
      else {
        this.type_select = '';
      }
      if (initObj.hasOwnProperty('turn_velocity')) {
        this.turn_velocity = initObj.turn_velocity
      }
      else {
        this.turn_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('move_velocity')) {
        this.move_velocity = initObj.move_velocity
      }
      else {
        this.move_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('total_time')) {
        this.total_time = initObj.total_time
      }
      else {
        this.total_time = 0.0;
      }
      if (initObj.hasOwnProperty('square_length')) {
        this.square_length = initObj.square_length
      }
      else {
        this.square_length = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type script_select
    // Serialize message field [script_select]
    bufferOffset = _serializer.string(obj.script_select, buffer, bufferOffset);
    // Serialize message field [type_select]
    bufferOffset = _serializer.string(obj.type_select, buffer, bufferOffset);
    // Serialize message field [turn_velocity]
    bufferOffset = _serializer.float32(obj.turn_velocity, buffer, bufferOffset);
    // Serialize message field [move_velocity]
    bufferOffset = _serializer.float32(obj.move_velocity, buffer, bufferOffset);
    // Serialize message field [total_time]
    bufferOffset = _serializer.float32(obj.total_time, buffer, bufferOffset);
    // Serialize message field [square_length]
    bufferOffset = _serializer.float32(obj.square_length, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type script_select
    let len;
    let data = new script_select(null);
    // Deserialize message field [script_select]
    data.script_select = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [type_select]
    data.type_select = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [turn_velocity]
    data.turn_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [move_velocity]
    data.move_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [total_time]
    data.total_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [square_length]
    data.square_length = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.script_select.length;
    length += object.type_select.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'open_path_follower/script_select';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dde5cb34cae0c7a4f86426afcd558fdc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string script_select
    string type_select
    float32 turn_velocity
    float32 move_velocity
    float32 total_time
    float32 square_length
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new script_select(null);
    if (msg.script_select !== undefined) {
      resolved.script_select = msg.script_select;
    }
    else {
      resolved.script_select = ''
    }

    if (msg.type_select !== undefined) {
      resolved.type_select = msg.type_select;
    }
    else {
      resolved.type_select = ''
    }

    if (msg.turn_velocity !== undefined) {
      resolved.turn_velocity = msg.turn_velocity;
    }
    else {
      resolved.turn_velocity = 0.0
    }

    if (msg.move_velocity !== undefined) {
      resolved.move_velocity = msg.move_velocity;
    }
    else {
      resolved.move_velocity = 0.0
    }

    if (msg.total_time !== undefined) {
      resolved.total_time = msg.total_time;
    }
    else {
      resolved.total_time = 0.0
    }

    if (msg.square_length !== undefined) {
      resolved.square_length = msg.square_length;
    }
    else {
      resolved.square_length = 0.0
    }

    return resolved;
    }
};

module.exports = script_select;
