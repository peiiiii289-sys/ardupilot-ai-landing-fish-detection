/** @file
 *    @brief MAVLink comm protocol testsuite generated from flightstack.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef FLIGHTSTACK_TESTSUITE_H
#define FLIGHTSTACK_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_flightstack(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_flightstack(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_ai_landing_correction(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AI_LANDING_CORRECTION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ai_landing_correction_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,113,180
    };
    mavlink_ai_landing_correction_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.roll_err = packet_in.roll_err;
        packet1.pitch_err = packet_in.pitch_err;
        packet1.yaw_err = packet_in.yaw_err;
        packet1.x_err = packet_in.x_err;
        packet1.y_err = packet_in.y_err;
        packet1.z_err = packet_in.z_err;
        packet1.distance = packet_in.distance;
        packet1.confidence = packet_in.confidence;
        packet1.frame = packet_in.frame;
        packet1.flags = packet_in.flags;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AI_LANDING_CORRECTION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ai_landing_correction_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ai_landing_correction_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ai_landing_correction_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.roll_err , packet1.pitch_err , packet1.yaw_err , packet1.x_err , packet1.y_err , packet1.z_err , packet1.distance , packet1.confidence , packet1.frame , packet1.flags );
    mavlink_msg_ai_landing_correction_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ai_landing_correction_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.roll_err , packet1.pitch_err , packet1.yaw_err , packet1.x_err , packet1.y_err , packet1.z_err , packet1.distance , packet1.confidence , packet1.frame , packet1.flags );
    mavlink_msg_ai_landing_correction_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ai_landing_correction_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ai_landing_correction_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.roll_err , packet1.pitch_err , packet1.yaw_err , packet1.x_err , packet1.y_err , packet1.z_err , packet1.distance , packet1.confidence , packet1.frame , packet1.flags );
    mavlink_msg_ai_landing_correction_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AI_LANDING_CORRECTION") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AI_LANDING_CORRECTION) != NULL);
#endif
}

static void mavlink_test_ai_landing_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AI_LANDING_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ai_landing_status_t packet_in = {
        963497464,45.0,73.0,101.0,53
    };
    mavlink_ai_landing_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.visual_confidence = packet_in.visual_confidence;
        packet1.reproj_error = packet_in.reproj_error;
        packet1.covariance = packet_in.covariance;
        packet1.target_lost = packet_in.target_lost;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AI_LANDING_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AI_LANDING_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ai_landing_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ai_landing_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ai_landing_status_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.visual_confidence , packet1.target_lost , packet1.reproj_error , packet1.covariance );
    mavlink_msg_ai_landing_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ai_landing_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.visual_confidence , packet1.target_lost , packet1.reproj_error , packet1.covariance );
    mavlink_msg_ai_landing_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ai_landing_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ai_landing_status_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.visual_confidence , packet1.target_lost , packet1.reproj_error , packet1.covariance );
    mavlink_msg_ai_landing_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("AI_LANDING_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_AI_LANDING_STATUS) != NULL);
#endif
}

static void mavlink_test_flightstack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ai_landing_correction(system_id, component_id, last_msg);
    mavlink_test_ai_landing_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // FLIGHTSTACK_TESTSUITE_H
