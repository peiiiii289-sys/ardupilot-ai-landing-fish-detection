#pragma once
// MESSAGE AI_LANDING_CORRECTION PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_AI_LANDING_CORRECTION 52100


typedef struct __mavlink_ai_landing_correction_t {
 uint32_t time_boot_ms; /*<  Timestamp since AI boot*/
 float roll_err; /*<  Roll correction in rad*/
 float pitch_err; /*<  Pitch correction in rad*/
 float yaw_err; /*<  Yaw correction in rad*/
 float x_err; /*<  Forward/backward error in m*/
 float y_err; /*<  Right/left error in m*/
 float z_err; /*<  Down/up error in m*/
 float distance; /*<  Distance to landing point in m*/
 float confidence; /*<  0-1 confidence*/
 uint8_t frame; /*<  AI_LANDING_FRAME*/
 uint8_t flags; /*<  bit0=yaw_valid bit1=distance_valid*/
} mavlink_ai_landing_correction_t;

#define MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN 38
#define MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN 38
#define MAVLINK_MSG_ID_52100_LEN 38
#define MAVLINK_MSG_ID_52100_MIN_LEN 38

#define MAVLINK_MSG_ID_AI_LANDING_CORRECTION_CRC 83
#define MAVLINK_MSG_ID_52100_CRC 83



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AI_LANDING_CORRECTION { \
    52100, \
    "AI_LANDING_CORRECTION", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ai_landing_correction_t, time_boot_ms) }, \
         { "roll_err", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ai_landing_correction_t, roll_err) }, \
         { "pitch_err", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ai_landing_correction_t, pitch_err) }, \
         { "yaw_err", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ai_landing_correction_t, yaw_err) }, \
         { "x_err", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ai_landing_correction_t, x_err) }, \
         { "y_err", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ai_landing_correction_t, y_err) }, \
         { "z_err", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ai_landing_correction_t, z_err) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ai_landing_correction_t, distance) }, \
         { "confidence", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ai_landing_correction_t, confidence) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_ai_landing_correction_t, frame) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_ai_landing_correction_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AI_LANDING_CORRECTION { \
    "AI_LANDING_CORRECTION", \
    11, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ai_landing_correction_t, time_boot_ms) }, \
         { "roll_err", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ai_landing_correction_t, roll_err) }, \
         { "pitch_err", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ai_landing_correction_t, pitch_err) }, \
         { "yaw_err", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ai_landing_correction_t, yaw_err) }, \
         { "x_err", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ai_landing_correction_t, x_err) }, \
         { "y_err", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ai_landing_correction_t, y_err) }, \
         { "z_err", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ai_landing_correction_t, z_err) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ai_landing_correction_t, distance) }, \
         { "confidence", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ai_landing_correction_t, confidence) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_ai_landing_correction_t, frame) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_ai_landing_correction_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a ai_landing_correction message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms  Timestamp since AI boot
 * @param roll_err  Roll correction in rad
 * @param pitch_err  Pitch correction in rad
 * @param yaw_err  Yaw correction in rad
 * @param x_err  Forward/backward error in m
 * @param y_err  Right/left error in m
 * @param z_err  Down/up error in m
 * @param distance  Distance to landing point in m
 * @param confidence  0-1 confidence
 * @param frame  AI_LANDING_FRAME
 * @param flags  bit0=yaw_valid bit1=distance_valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ai_landing_correction_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float roll_err, float pitch_err, float yaw_err, float x_err, float y_err, float z_err, float distance, float confidence, uint8_t frame, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll_err);
    _mav_put_float(buf, 8, pitch_err);
    _mav_put_float(buf, 12, yaw_err);
    _mav_put_float(buf, 16, x_err);
    _mav_put_float(buf, 20, y_err);
    _mav_put_float(buf, 24, z_err);
    _mav_put_float(buf, 28, distance);
    _mav_put_float(buf, 32, confidence);
    _mav_put_uint8_t(buf, 36, frame);
    _mav_put_uint8_t(buf, 37, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN);
#else
    mavlink_ai_landing_correction_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll_err = roll_err;
    packet.pitch_err = pitch_err;
    packet.yaw_err = yaw_err;
    packet.x_err = x_err;
    packet.y_err = y_err;
    packet.z_err = z_err;
    packet.distance = distance;
    packet.confidence = confidence;
    packet.frame = frame;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AI_LANDING_CORRECTION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_CRC);
}

/**
 * @brief Pack a ai_landing_correction message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms  Timestamp since AI boot
 * @param roll_err  Roll correction in rad
 * @param pitch_err  Pitch correction in rad
 * @param yaw_err  Yaw correction in rad
 * @param x_err  Forward/backward error in m
 * @param y_err  Right/left error in m
 * @param z_err  Down/up error in m
 * @param distance  Distance to landing point in m
 * @param confidence  0-1 confidence
 * @param frame  AI_LANDING_FRAME
 * @param flags  bit0=yaw_valid bit1=distance_valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ai_landing_correction_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float roll_err, float pitch_err, float yaw_err, float x_err, float y_err, float z_err, float distance, float confidence, uint8_t frame, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll_err);
    _mav_put_float(buf, 8, pitch_err);
    _mav_put_float(buf, 12, yaw_err);
    _mav_put_float(buf, 16, x_err);
    _mav_put_float(buf, 20, y_err);
    _mav_put_float(buf, 24, z_err);
    _mav_put_float(buf, 28, distance);
    _mav_put_float(buf, 32, confidence);
    _mav_put_uint8_t(buf, 36, frame);
    _mav_put_uint8_t(buf, 37, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN);
#else
    mavlink_ai_landing_correction_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll_err = roll_err;
    packet.pitch_err = pitch_err;
    packet.yaw_err = yaw_err;
    packet.x_err = x_err;
    packet.y_err = y_err;
    packet.z_err = z_err;
    packet.distance = distance;
    packet.confidence = confidence;
    packet.frame = frame;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AI_LANDING_CORRECTION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN);
#endif
}

/**
 * @brief Pack a ai_landing_correction message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms  Timestamp since AI boot
 * @param roll_err  Roll correction in rad
 * @param pitch_err  Pitch correction in rad
 * @param yaw_err  Yaw correction in rad
 * @param x_err  Forward/backward error in m
 * @param y_err  Right/left error in m
 * @param z_err  Down/up error in m
 * @param distance  Distance to landing point in m
 * @param confidence  0-1 confidence
 * @param frame  AI_LANDING_FRAME
 * @param flags  bit0=yaw_valid bit1=distance_valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ai_landing_correction_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float roll_err,float pitch_err,float yaw_err,float x_err,float y_err,float z_err,float distance,float confidence,uint8_t frame,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll_err);
    _mav_put_float(buf, 8, pitch_err);
    _mav_put_float(buf, 12, yaw_err);
    _mav_put_float(buf, 16, x_err);
    _mav_put_float(buf, 20, y_err);
    _mav_put_float(buf, 24, z_err);
    _mav_put_float(buf, 28, distance);
    _mav_put_float(buf, 32, confidence);
    _mav_put_uint8_t(buf, 36, frame);
    _mav_put_uint8_t(buf, 37, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN);
#else
    mavlink_ai_landing_correction_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll_err = roll_err;
    packet.pitch_err = pitch_err;
    packet.yaw_err = yaw_err;
    packet.x_err = x_err;
    packet.y_err = y_err;
    packet.z_err = z_err;
    packet.distance = distance;
    packet.confidence = confidence;
    packet.frame = frame;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AI_LANDING_CORRECTION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_CRC);
}

/**
 * @brief Encode a ai_landing_correction struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ai_landing_correction C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ai_landing_correction_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ai_landing_correction_t* ai_landing_correction)
{
    return mavlink_msg_ai_landing_correction_pack(system_id, component_id, msg, ai_landing_correction->time_boot_ms, ai_landing_correction->roll_err, ai_landing_correction->pitch_err, ai_landing_correction->yaw_err, ai_landing_correction->x_err, ai_landing_correction->y_err, ai_landing_correction->z_err, ai_landing_correction->distance, ai_landing_correction->confidence, ai_landing_correction->frame, ai_landing_correction->flags);
}

/**
 * @brief Encode a ai_landing_correction struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ai_landing_correction C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ai_landing_correction_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ai_landing_correction_t* ai_landing_correction)
{
    return mavlink_msg_ai_landing_correction_pack_chan(system_id, component_id, chan, msg, ai_landing_correction->time_boot_ms, ai_landing_correction->roll_err, ai_landing_correction->pitch_err, ai_landing_correction->yaw_err, ai_landing_correction->x_err, ai_landing_correction->y_err, ai_landing_correction->z_err, ai_landing_correction->distance, ai_landing_correction->confidence, ai_landing_correction->frame, ai_landing_correction->flags);
}

/**
 * @brief Encode a ai_landing_correction struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param ai_landing_correction C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ai_landing_correction_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_ai_landing_correction_t* ai_landing_correction)
{
    return mavlink_msg_ai_landing_correction_pack_status(system_id, component_id, _status, msg,  ai_landing_correction->time_boot_ms, ai_landing_correction->roll_err, ai_landing_correction->pitch_err, ai_landing_correction->yaw_err, ai_landing_correction->x_err, ai_landing_correction->y_err, ai_landing_correction->z_err, ai_landing_correction->distance, ai_landing_correction->confidence, ai_landing_correction->frame, ai_landing_correction->flags);
}

/**
 * @brief Send a ai_landing_correction message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms  Timestamp since AI boot
 * @param roll_err  Roll correction in rad
 * @param pitch_err  Pitch correction in rad
 * @param yaw_err  Yaw correction in rad
 * @param x_err  Forward/backward error in m
 * @param y_err  Right/left error in m
 * @param z_err  Down/up error in m
 * @param distance  Distance to landing point in m
 * @param confidence  0-1 confidence
 * @param frame  AI_LANDING_FRAME
 * @param flags  bit0=yaw_valid bit1=distance_valid
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ai_landing_correction_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll_err, float pitch_err, float yaw_err, float x_err, float y_err, float z_err, float distance, float confidence, uint8_t frame, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll_err);
    _mav_put_float(buf, 8, pitch_err);
    _mav_put_float(buf, 12, yaw_err);
    _mav_put_float(buf, 16, x_err);
    _mav_put_float(buf, 20, y_err);
    _mav_put_float(buf, 24, z_err);
    _mav_put_float(buf, 28, distance);
    _mav_put_float(buf, 32, confidence);
    _mav_put_uint8_t(buf, 36, frame);
    _mav_put_uint8_t(buf, 37, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AI_LANDING_CORRECTION, buf, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_CRC);
#else
    mavlink_ai_landing_correction_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.roll_err = roll_err;
    packet.pitch_err = pitch_err;
    packet.yaw_err = yaw_err;
    packet.x_err = x_err;
    packet.y_err = y_err;
    packet.z_err = z_err;
    packet.distance = distance;
    packet.confidence = confidence;
    packet.frame = frame;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AI_LANDING_CORRECTION, (const char *)&packet, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_CRC);
#endif
}

/**
 * @brief Send a ai_landing_correction message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ai_landing_correction_send_struct(mavlink_channel_t chan, const mavlink_ai_landing_correction_t* ai_landing_correction)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ai_landing_correction_send(chan, ai_landing_correction->time_boot_ms, ai_landing_correction->roll_err, ai_landing_correction->pitch_err, ai_landing_correction->yaw_err, ai_landing_correction->x_err, ai_landing_correction->y_err, ai_landing_correction->z_err, ai_landing_correction->distance, ai_landing_correction->confidence, ai_landing_correction->frame, ai_landing_correction->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AI_LANDING_CORRECTION, (const char *)ai_landing_correction, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_CRC);
#endif
}

#if MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ai_landing_correction_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float roll_err, float pitch_err, float yaw_err, float x_err, float y_err, float z_err, float distance, float confidence, uint8_t frame, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, roll_err);
    _mav_put_float(buf, 8, pitch_err);
    _mav_put_float(buf, 12, yaw_err);
    _mav_put_float(buf, 16, x_err);
    _mav_put_float(buf, 20, y_err);
    _mav_put_float(buf, 24, z_err);
    _mav_put_float(buf, 28, distance);
    _mav_put_float(buf, 32, confidence);
    _mav_put_uint8_t(buf, 36, frame);
    _mav_put_uint8_t(buf, 37, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AI_LANDING_CORRECTION, buf, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_CRC);
#else
    mavlink_ai_landing_correction_t *packet = (mavlink_ai_landing_correction_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->roll_err = roll_err;
    packet->pitch_err = pitch_err;
    packet->yaw_err = yaw_err;
    packet->x_err = x_err;
    packet->y_err = y_err;
    packet->z_err = z_err;
    packet->distance = distance;
    packet->confidence = confidence;
    packet->frame = frame;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AI_LANDING_CORRECTION, (const char *)packet, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_CRC);
#endif
}
#endif

#endif

// MESSAGE AI_LANDING_CORRECTION UNPACKING


/**
 * @brief Get field time_boot_ms from ai_landing_correction message
 *
 * @return  Timestamp since AI boot
 */
static inline uint32_t mavlink_msg_ai_landing_correction_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field roll_err from ai_landing_correction message
 *
 * @return  Roll correction in rad
 */
static inline float mavlink_msg_ai_landing_correction_get_roll_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch_err from ai_landing_correction message
 *
 * @return  Pitch correction in rad
 */
static inline float mavlink_msg_ai_landing_correction_get_pitch_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw_err from ai_landing_correction message
 *
 * @return  Yaw correction in rad
 */
static inline float mavlink_msg_ai_landing_correction_get_yaw_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field x_err from ai_landing_correction message
 *
 * @return  Forward/backward error in m
 */
static inline float mavlink_msg_ai_landing_correction_get_x_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field y_err from ai_landing_correction message
 *
 * @return  Right/left error in m
 */
static inline float mavlink_msg_ai_landing_correction_get_y_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field z_err from ai_landing_correction message
 *
 * @return  Down/up error in m
 */
static inline float mavlink_msg_ai_landing_correction_get_z_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field distance from ai_landing_correction message
 *
 * @return  Distance to landing point in m
 */
static inline float mavlink_msg_ai_landing_correction_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field confidence from ai_landing_correction message
 *
 * @return  0-1 confidence
 */
static inline float mavlink_msg_ai_landing_correction_get_confidence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field frame from ai_landing_correction message
 *
 * @return  AI_LANDING_FRAME
 */
static inline uint8_t mavlink_msg_ai_landing_correction_get_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field flags from ai_landing_correction message
 *
 * @return  bit0=yaw_valid bit1=distance_valid
 */
static inline uint8_t mavlink_msg_ai_landing_correction_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Decode a ai_landing_correction message into a struct
 *
 * @param msg The message to decode
 * @param ai_landing_correction C-struct to decode the message contents into
 */
static inline void mavlink_msg_ai_landing_correction_decode(const mavlink_message_t* msg, mavlink_ai_landing_correction_t* ai_landing_correction)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ai_landing_correction->time_boot_ms = mavlink_msg_ai_landing_correction_get_time_boot_ms(msg);
    ai_landing_correction->roll_err = mavlink_msg_ai_landing_correction_get_roll_err(msg);
    ai_landing_correction->pitch_err = mavlink_msg_ai_landing_correction_get_pitch_err(msg);
    ai_landing_correction->yaw_err = mavlink_msg_ai_landing_correction_get_yaw_err(msg);
    ai_landing_correction->x_err = mavlink_msg_ai_landing_correction_get_x_err(msg);
    ai_landing_correction->y_err = mavlink_msg_ai_landing_correction_get_y_err(msg);
    ai_landing_correction->z_err = mavlink_msg_ai_landing_correction_get_z_err(msg);
    ai_landing_correction->distance = mavlink_msg_ai_landing_correction_get_distance(msg);
    ai_landing_correction->confidence = mavlink_msg_ai_landing_correction_get_confidence(msg);
    ai_landing_correction->frame = mavlink_msg_ai_landing_correction_get_frame(msg);
    ai_landing_correction->flags = mavlink_msg_ai_landing_correction_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN? msg->len : MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN;
        memset(ai_landing_correction, 0, MAVLINK_MSG_ID_AI_LANDING_CORRECTION_LEN);
    memcpy(ai_landing_correction, _MAV_PAYLOAD(msg), len);
#endif
}
