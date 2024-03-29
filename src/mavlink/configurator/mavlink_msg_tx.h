#pragma once
// MESSAGE tx PACKING

#define MAVLINK_MSG_ID_tx 2

MAVPACKED(
typedef struct __mavlink_tx_t {
 int16_t roll; /*<  roll*/
 int16_t pitch; /*<  pitch*/
 int16_t yaw; /*<  yaw*/
 uint16_t throttle; /*<  throttle*/
 uint16_t aux1; /*<  aux1*/
 uint16_t aux2; /*<  aux2*/
 uint16_t aux3; /*<  aux3*/
 uint16_t aux4; /*<  aux4*/
 int8_t anglex; /*<  anglex*/
 int8_t angley; /*<  anglex*/
 int8_t anglez; /*<  anglex*/
}) mavlink_tx_t;

#define MAVLINK_MSG_ID_tx_LEN 19
#define MAVLINK_MSG_ID_tx_MIN_LEN 19
#define MAVLINK_MSG_ID_2_LEN 19
#define MAVLINK_MSG_ID_2_MIN_LEN 19

#define MAVLINK_MSG_ID_tx_CRC 176
#define MAVLINK_MSG_ID_2_CRC 176



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_tx { \
    2, \
    "tx", \
    11, \
    {  { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_tx_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_tx_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_tx_t, yaw) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_tx_t, throttle) }, \
         { "aux1", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_tx_t, aux1) }, \
         { "aux2", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_tx_t, aux2) }, \
         { "aux3", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_tx_t, aux3) }, \
         { "aux4", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_tx_t, aux4) }, \
         { "anglex", NULL, MAVLINK_TYPE_INT8_T, 0, 16, offsetof(mavlink_tx_t, anglex) }, \
         { "angley", NULL, MAVLINK_TYPE_INT8_T, 0, 17, offsetof(mavlink_tx_t, angley) }, \
         { "anglez", NULL, MAVLINK_TYPE_INT8_T, 0, 18, offsetof(mavlink_tx_t, anglez) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_tx { \
    "tx", \
    11, \
    {  { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_tx_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_tx_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_tx_t, yaw) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_tx_t, throttle) }, \
         { "aux1", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_tx_t, aux1) }, \
         { "aux2", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_tx_t, aux2) }, \
         { "aux3", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_tx_t, aux3) }, \
         { "aux4", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_tx_t, aux4) }, \
         { "anglex", NULL, MAVLINK_TYPE_INT8_T, 0, 16, offsetof(mavlink_tx_t, anglex) }, \
         { "angley", NULL, MAVLINK_TYPE_INT8_T, 0, 17, offsetof(mavlink_tx_t, angley) }, \
         { "anglez", NULL, MAVLINK_TYPE_INT8_T, 0, 18, offsetof(mavlink_tx_t, anglez) }, \
         } \
}
#endif

/**
 * @brief Pack a tx message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll  roll
 * @param pitch  pitch
 * @param yaw  yaw
 * @param throttle  throttle
 * @param aux1  aux1
 * @param aux2  aux2
 * @param aux3  aux3
 * @param aux4  aux4
 * @param anglex  anglex
 * @param angley  anglex
 * @param anglez  anglex
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tx_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t roll, int16_t pitch, int16_t yaw, uint16_t throttle, uint16_t aux1, uint16_t aux2, uint16_t aux3, uint16_t aux4, int8_t anglex, int8_t angley, int8_t anglez)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_tx_LEN];
    _mav_put_int16_t(buf, 0, roll);
    _mav_put_int16_t(buf, 2, pitch);
    _mav_put_int16_t(buf, 4, yaw);
    _mav_put_uint16_t(buf, 6, throttle);
    _mav_put_uint16_t(buf, 8, aux1);
    _mav_put_uint16_t(buf, 10, aux2);
    _mav_put_uint16_t(buf, 12, aux3);
    _mav_put_uint16_t(buf, 14, aux4);
    _mav_put_int8_t(buf, 16, anglex);
    _mav_put_int8_t(buf, 17, angley);
    _mav_put_int8_t(buf, 18, anglez);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_tx_LEN);
#else
    mavlink_tx_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.throttle = throttle;
    packet.aux1 = aux1;
    packet.aux2 = aux2;
    packet.aux3 = aux3;
    packet.aux4 = aux4;
    packet.anglex = anglex;
    packet.angley = angley;
    packet.anglez = anglez;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_tx_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_tx;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_tx_MIN_LEN, MAVLINK_MSG_ID_tx_LEN, MAVLINK_MSG_ID_tx_CRC);
}

/**
 * @brief Pack a tx message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll  roll
 * @param pitch  pitch
 * @param yaw  yaw
 * @param throttle  throttle
 * @param aux1  aux1
 * @param aux2  aux2
 * @param aux3  aux3
 * @param aux4  aux4
 * @param anglex  anglex
 * @param angley  anglex
 * @param anglez  anglex
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tx_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t roll,int16_t pitch,int16_t yaw,uint16_t throttle,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,int8_t anglex,int8_t angley,int8_t anglez)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_tx_LEN];
    _mav_put_int16_t(buf, 0, roll);
    _mav_put_int16_t(buf, 2, pitch);
    _mav_put_int16_t(buf, 4, yaw);
    _mav_put_uint16_t(buf, 6, throttle);
    _mav_put_uint16_t(buf, 8, aux1);
    _mav_put_uint16_t(buf, 10, aux2);
    _mav_put_uint16_t(buf, 12, aux3);
    _mav_put_uint16_t(buf, 14, aux4);
    _mav_put_int8_t(buf, 16, anglex);
    _mav_put_int8_t(buf, 17, angley);
    _mav_put_int8_t(buf, 18, anglez);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_tx_LEN);
#else
    mavlink_tx_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.throttle = throttle;
    packet.aux1 = aux1;
    packet.aux2 = aux2;
    packet.aux3 = aux3;
    packet.aux4 = aux4;
    packet.anglex = anglex;
    packet.angley = angley;
    packet.anglez = anglez;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_tx_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_tx;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_tx_MIN_LEN, MAVLINK_MSG_ID_tx_LEN, MAVLINK_MSG_ID_tx_CRC);
}

/**
 * @brief Encode a tx struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tx C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tx_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tx_t* tx)
{
    return mavlink_msg_tx_pack(system_id, component_id, msg, tx->roll, tx->pitch, tx->yaw, tx->throttle, tx->aux1, tx->aux2, tx->aux3, tx->aux4, tx->anglex, tx->angley, tx->anglez);
}

/**
 * @brief Encode a tx struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tx C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tx_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tx_t* tx)
{
    return mavlink_msg_tx_pack_chan(system_id, component_id, chan, msg, tx->roll, tx->pitch, tx->yaw, tx->throttle, tx->aux1, tx->aux2, tx->aux3, tx->aux4, tx->anglex, tx->angley, tx->anglez);
}

/**
 * @brief Send a tx message
 * @param chan MAVLink channel to send the message
 *
 * @param roll  roll
 * @param pitch  pitch
 * @param yaw  yaw
 * @param throttle  throttle
 * @param aux1  aux1
 * @param aux2  aux2
 * @param aux3  aux3
 * @param aux4  aux4
 * @param anglex  anglex
 * @param angley  anglex
 * @param anglez  anglex
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tx_send(mavlink_channel_t chan, int16_t roll, int16_t pitch, int16_t yaw, uint16_t throttle, uint16_t aux1, uint16_t aux2, uint16_t aux3, uint16_t aux4, int8_t anglex, int8_t angley, int8_t anglez)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_tx_LEN];
    _mav_put_int16_t(buf, 0, roll);
    _mav_put_int16_t(buf, 2, pitch);
    _mav_put_int16_t(buf, 4, yaw);
    _mav_put_uint16_t(buf, 6, throttle);
    _mav_put_uint16_t(buf, 8, aux1);
    _mav_put_uint16_t(buf, 10, aux2);
    _mav_put_uint16_t(buf, 12, aux3);
    _mav_put_uint16_t(buf, 14, aux4);
    _mav_put_int8_t(buf, 16, anglex);
    _mav_put_int8_t(buf, 17, angley);
    _mav_put_int8_t(buf, 18, anglez);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_tx, buf, MAVLINK_MSG_ID_tx_MIN_LEN, MAVLINK_MSG_ID_tx_LEN, MAVLINK_MSG_ID_tx_CRC);
#else
    mavlink_tx_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.throttle = throttle;
    packet.aux1 = aux1;
    packet.aux2 = aux2;
    packet.aux3 = aux3;
    packet.aux4 = aux4;
    packet.anglex = anglex;
    packet.angley = angley;
    packet.anglez = anglez;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_tx, (const char *)&packet, MAVLINK_MSG_ID_tx_MIN_LEN, MAVLINK_MSG_ID_tx_LEN, MAVLINK_MSG_ID_tx_CRC);
#endif
}

/**
 * @brief Send a tx message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tx_send_struct(mavlink_channel_t chan, const mavlink_tx_t* tx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tx_send(chan, tx->roll, tx->pitch, tx->yaw, tx->throttle, tx->aux1, tx->aux2, tx->aux3, tx->aux4, tx->anglex, tx->angley, tx->anglez);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_tx, (const char *)tx, MAVLINK_MSG_ID_tx_MIN_LEN, MAVLINK_MSG_ID_tx_LEN, MAVLINK_MSG_ID_tx_CRC);
#endif
}

#if MAVLINK_MSG_ID_tx_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tx_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t roll, int16_t pitch, int16_t yaw, uint16_t throttle, uint16_t aux1, uint16_t aux2, uint16_t aux3, uint16_t aux4, int8_t anglex, int8_t angley, int8_t anglez)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, roll);
    _mav_put_int16_t(buf, 2, pitch);
    _mav_put_int16_t(buf, 4, yaw);
    _mav_put_uint16_t(buf, 6, throttle);
    _mav_put_uint16_t(buf, 8, aux1);
    _mav_put_uint16_t(buf, 10, aux2);
    _mav_put_uint16_t(buf, 12, aux3);
    _mav_put_uint16_t(buf, 14, aux4);
    _mav_put_int8_t(buf, 16, anglex);
    _mav_put_int8_t(buf, 17, angley);
    _mav_put_int8_t(buf, 18, anglez);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_tx, buf, MAVLINK_MSG_ID_tx_MIN_LEN, MAVLINK_MSG_ID_tx_LEN, MAVLINK_MSG_ID_tx_CRC);
#else
    mavlink_tx_t *packet = (mavlink_tx_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->throttle = throttle;
    packet->aux1 = aux1;
    packet->aux2 = aux2;
    packet->aux3 = aux3;
    packet->aux4 = aux4;
    packet->anglex = anglex;
    packet->angley = angley;
    packet->anglez = anglez;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_tx, (const char *)packet, MAVLINK_MSG_ID_tx_MIN_LEN, MAVLINK_MSG_ID_tx_LEN, MAVLINK_MSG_ID_tx_CRC);
#endif
}
#endif

#endif

// MESSAGE tx UNPACKING


/**
 * @brief Get field roll from tx message
 *
 * @return  roll
 */
static inline int16_t mavlink_msg_tx_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field pitch from tx message
 *
 * @return  pitch
 */
static inline int16_t mavlink_msg_tx_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field yaw from tx message
 *
 * @return  yaw
 */
static inline int16_t mavlink_msg_tx_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field throttle from tx message
 *
 * @return  throttle
 */
static inline uint16_t mavlink_msg_tx_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field aux1 from tx message
 *
 * @return  aux1
 */
static inline uint16_t mavlink_msg_tx_get_aux1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field aux2 from tx message
 *
 * @return  aux2
 */
static inline uint16_t mavlink_msg_tx_get_aux2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field aux3 from tx message
 *
 * @return  aux3
 */
static inline uint16_t mavlink_msg_tx_get_aux3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field aux4 from tx message
 *
 * @return  aux4
 */
static inline uint16_t mavlink_msg_tx_get_aux4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field anglex from tx message
 *
 * @return  anglex
 */
static inline int8_t mavlink_msg_tx_get_anglex(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  16);
}

/**
 * @brief Get field angley from tx message
 *
 * @return  anglex
 */
static inline int8_t mavlink_msg_tx_get_angley(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  17);
}

/**
 * @brief Get field anglez from tx message
 *
 * @return  anglex
 */
static inline int8_t mavlink_msg_tx_get_anglez(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  18);
}

/**
 * @brief Decode a tx message into a struct
 *
 * @param msg The message to decode
 * @param tx C-struct to decode the message contents into
 */
static inline void mavlink_msg_tx_decode(const mavlink_message_t* msg, mavlink_tx_t* tx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tx->roll = mavlink_msg_tx_get_roll(msg);
    tx->pitch = mavlink_msg_tx_get_pitch(msg);
    tx->yaw = mavlink_msg_tx_get_yaw(msg);
    tx->throttle = mavlink_msg_tx_get_throttle(msg);
    tx->aux1 = mavlink_msg_tx_get_aux1(msg);
    tx->aux2 = mavlink_msg_tx_get_aux2(msg);
    tx->aux3 = mavlink_msg_tx_get_aux3(msg);
    tx->aux4 = mavlink_msg_tx_get_aux4(msg);
    tx->anglex = mavlink_msg_tx_get_anglex(msg);
    tx->angley = mavlink_msg_tx_get_angley(msg);
    tx->anglez = mavlink_msg_tx_get_anglez(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_tx_LEN? msg->len : MAVLINK_MSG_ID_tx_LEN;
        memset(tx, 0, MAVLINK_MSG_ID_tx_LEN);
    memcpy(tx, _MAV_PAYLOAD(msg), len);
#endif
}
