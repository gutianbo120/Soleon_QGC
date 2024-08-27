#pragma once
// MESSAGE SO_STATUS PACKING

#define MAVLINK_MSG_ID_SO_STATUS 50080


typedef struct __mavlink_so_status_t {
 uint32_t timestamp; /*< [ms] Timestamp.*/
 float filllevel; /*< [l] Fill level liter*/
 float flowliter; /*< [l/min] Flow rate liter/min*/
 float flowha; /*< [l/ha] Flow rate liter/ha.*/
 float distlines; /*< [m] Distance between lines.*/
 float speed; /*< [m/s] Speed of the mission.*/
 uint8_t status; /*< [bitfield] Status byte,b0:Spray right, b1:Spray left,b2:ready,b3:pump erro,b4:nozzles error*/
} mavlink_so_status_t;

#define MAVLINK_MSG_ID_SO_STATUS_LEN 25
#define MAVLINK_MSG_ID_SO_STATUS_MIN_LEN 25
#define MAVLINK_MSG_ID_50080_LEN 25
#define MAVLINK_MSG_ID_50080_MIN_LEN 25

#define MAVLINK_MSG_ID_SO_STATUS_CRC 89
#define MAVLINK_MSG_ID_50080_CRC 89



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SO_STATUS { \
    50080, \
    "SO_STATUS", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_so_status_t, timestamp) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_so_status_t, status) }, \
         { "filllevel", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_so_status_t, filllevel) }, \
         { "flowliter", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_so_status_t, flowliter) }, \
         { "flowha", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_so_status_t, flowha) }, \
         { "distlines", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_so_status_t, distlines) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_so_status_t, speed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SO_STATUS { \
    "SO_STATUS", \
    7, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_so_status_t, timestamp) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_so_status_t, status) }, \
         { "filllevel", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_so_status_t, filllevel) }, \
         { "flowliter", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_so_status_t, flowliter) }, \
         { "flowha", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_so_status_t, flowha) }, \
         { "distlines", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_so_status_t, distlines) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_so_status_t, speed) }, \
         } \
}
#endif

/**
 * @brief Pack a so_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp.
 * @param status [bitfield] Status byte,b0:Spray right, b1:Spray left,b2:ready,b3:pump erro,b4:nozzles error
 * @param filllevel [l] Fill level liter
 * @param flowliter [l/min] Flow rate liter/min
 * @param flowha [l/ha] Flow rate liter/ha.
 * @param distlines [m] Distance between lines.
 * @param speed [m/s] Speed of the mission.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_so_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t timestamp, uint8_t status, float filllevel, float flowliter, float flowha, float distlines, float speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SO_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_float(buf, 4, filllevel);
    _mav_put_float(buf, 8, flowliter);
    _mav_put_float(buf, 12, flowha);
    _mav_put_float(buf, 16, distlines);
    _mav_put_float(buf, 20, speed);
    _mav_put_uint8_t(buf, 24, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SO_STATUS_LEN);
#else
    mavlink_so_status_t packet;
    packet.timestamp = timestamp;
    packet.filllevel = filllevel;
    packet.flowliter = flowliter;
    packet.flowha = flowha;
    packet.distlines = distlines;
    packet.speed = speed;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SO_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SO_STATUS_MIN_LEN, MAVLINK_MSG_ID_SO_STATUS_LEN, MAVLINK_MSG_ID_SO_STATUS_CRC);
}

/**
 * @brief Pack a so_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] Timestamp.
 * @param status [bitfield] Status byte,b0:Spray right, b1:Spray left,b2:ready,b3:pump erro,b4:nozzles error
 * @param filllevel [l] Fill level liter
 * @param flowliter [l/min] Flow rate liter/min
 * @param flowha [l/ha] Flow rate liter/ha.
 * @param distlines [m] Distance between lines.
 * @param speed [m/s] Speed of the mission.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_so_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t timestamp,uint8_t status,float filllevel,float flowliter,float flowha,float distlines,float speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SO_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_float(buf, 4, filllevel);
    _mav_put_float(buf, 8, flowliter);
    _mav_put_float(buf, 12, flowha);
    _mav_put_float(buf, 16, distlines);
    _mav_put_float(buf, 20, speed);
    _mav_put_uint8_t(buf, 24, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SO_STATUS_LEN);
#else
    mavlink_so_status_t packet;
    packet.timestamp = timestamp;
    packet.filllevel = filllevel;
    packet.flowliter = flowliter;
    packet.flowha = flowha;
    packet.distlines = distlines;
    packet.speed = speed;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SO_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SO_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SO_STATUS_MIN_LEN, MAVLINK_MSG_ID_SO_STATUS_LEN, MAVLINK_MSG_ID_SO_STATUS_CRC);
}

/**
 * @brief Encode a so_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param so_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_so_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_so_status_t* so_status)
{
    return mavlink_msg_so_status_pack(system_id, component_id, msg, so_status->timestamp, so_status->status, so_status->filllevel, so_status->flowliter, so_status->flowha, so_status->distlines, so_status->speed);
}

/**
 * @brief Encode a so_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param so_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_so_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_so_status_t* so_status)
{
    return mavlink_msg_so_status_pack_chan(system_id, component_id, chan, msg, so_status->timestamp, so_status->status, so_status->filllevel, so_status->flowliter, so_status->flowha, so_status->distlines, so_status->speed);
}

/**
 * @brief Send a so_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] Timestamp.
 * @param status [bitfield] Status byte,b0:Spray right, b1:Spray left,b2:ready,b3:pump erro,b4:nozzles error
 * @param filllevel [l] Fill level liter
 * @param flowliter [l/min] Flow rate liter/min
 * @param flowha [l/ha] Flow rate liter/ha.
 * @param distlines [m] Distance between lines.
 * @param speed [m/s] Speed of the mission.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_so_status_send(mavlink_channel_t chan, uint32_t timestamp, uint8_t status, float filllevel, float flowliter, float flowha, float distlines, float speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SO_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_float(buf, 4, filllevel);
    _mav_put_float(buf, 8, flowliter);
    _mav_put_float(buf, 12, flowha);
    _mav_put_float(buf, 16, distlines);
    _mav_put_float(buf, 20, speed);
    _mav_put_uint8_t(buf, 24, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SO_STATUS, buf, MAVLINK_MSG_ID_SO_STATUS_MIN_LEN, MAVLINK_MSG_ID_SO_STATUS_LEN, MAVLINK_MSG_ID_SO_STATUS_CRC);
#else
    mavlink_so_status_t packet;
    packet.timestamp = timestamp;
    packet.filllevel = filllevel;
    packet.flowliter = flowliter;
    packet.flowha = flowha;
    packet.distlines = distlines;
    packet.speed = speed;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SO_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SO_STATUS_MIN_LEN, MAVLINK_MSG_ID_SO_STATUS_LEN, MAVLINK_MSG_ID_SO_STATUS_CRC);
#endif
}

/**
 * @brief Send a so_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_so_status_send_struct(mavlink_channel_t chan, const mavlink_so_status_t* so_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_so_status_send(chan, so_status->timestamp, so_status->status, so_status->filllevel, so_status->flowliter, so_status->flowha, so_status->distlines, so_status->speed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SO_STATUS, (const char *)so_status, MAVLINK_MSG_ID_SO_STATUS_MIN_LEN, MAVLINK_MSG_ID_SO_STATUS_LEN, MAVLINK_MSG_ID_SO_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SO_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_so_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t timestamp, uint8_t status, float filllevel, float flowliter, float flowha, float distlines, float speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, timestamp);
    _mav_put_float(buf, 4, filllevel);
    _mav_put_float(buf, 8, flowliter);
    _mav_put_float(buf, 12, flowha);
    _mav_put_float(buf, 16, distlines);
    _mav_put_float(buf, 20, speed);
    _mav_put_uint8_t(buf, 24, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SO_STATUS, buf, MAVLINK_MSG_ID_SO_STATUS_MIN_LEN, MAVLINK_MSG_ID_SO_STATUS_LEN, MAVLINK_MSG_ID_SO_STATUS_CRC);
#else
    mavlink_so_status_t *packet = (mavlink_so_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->filllevel = filllevel;
    packet->flowliter = flowliter;
    packet->flowha = flowha;
    packet->distlines = distlines;
    packet->speed = speed;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SO_STATUS, (const char *)packet, MAVLINK_MSG_ID_SO_STATUS_MIN_LEN, MAVLINK_MSG_ID_SO_STATUS_LEN, MAVLINK_MSG_ID_SO_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SO_STATUS UNPACKING


/**
 * @brief Get field timestamp from so_status message
 *
 * @return [ms] Timestamp.
 */
static inline uint32_t mavlink_msg_so_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field status from so_status message
 *
 * @return [bitfield] Status byte,b0:Spray right, b1:Spray left,b2:ready,b3:pump erro,b4:nozzles error
 */
static inline uint8_t mavlink_msg_so_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field filllevel from so_status message
 *
 * @return [l] Fill level liter
 */
static inline float mavlink_msg_so_status_get_filllevel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field flowliter from so_status message
 *
 * @return [l/min] Flow rate liter/min
 */
static inline float mavlink_msg_so_status_get_flowliter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field flowha from so_status message
 *
 * @return [l/ha] Flow rate liter/ha.
 */
static inline float mavlink_msg_so_status_get_flowha(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field distlines from so_status message
 *
 * @return [m] Distance between lines.
 */
static inline float mavlink_msg_so_status_get_distlines(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field speed from so_status message
 *
 * @return [m/s] Speed of the mission.
 */
static inline float mavlink_msg_so_status_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a so_status message into a struct
 *
 * @param msg The message to decode
 * @param so_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_so_status_decode(const mavlink_message_t* msg, mavlink_so_status_t* so_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    so_status->timestamp = mavlink_msg_so_status_get_timestamp(msg);
    so_status->filllevel = mavlink_msg_so_status_get_filllevel(msg);
    so_status->flowliter = mavlink_msg_so_status_get_flowliter(msg);
    so_status->flowha = mavlink_msg_so_status_get_flowha(msg);
    so_status->distlines = mavlink_msg_so_status_get_distlines(msg);
    so_status->speed = mavlink_msg_so_status_get_speed(msg);
    so_status->status = mavlink_msg_so_status_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SO_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SO_STATUS_LEN;
        memset(so_status, 0, MAVLINK_MSG_ID_SO_STATUS_LEN);
    memcpy(so_status, _MAV_PAYLOAD(msg), len);
#endif
}
