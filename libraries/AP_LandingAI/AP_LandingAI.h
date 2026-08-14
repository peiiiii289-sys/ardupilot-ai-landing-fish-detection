#include <GCS_MAVLink/GCS.h>
#include <stdint.h>
#pragma once

class AP_LandingAI
{
public:

    float l1_vzmax() const { return 0.30f; }
    float l1_damp() const { return 1.20f; }

    bool l1_warning() const { return _l1_warning; }
    bool l2_go_around() const { return _l2_go_around && !_l3_takeover; }
    bool l3_takeover() const { return _l3_takeover; }

    uint8_t level_reason() const { return _level_reason; }

    void reset_level_state()
    {
        _l1_warning = false;
        _l2_go_around = false;
        _l3_takeover = false;
        _level_reason = 0;
        _l1_since_ms = 0;
        _l2_last_trigger_ms = 0;
        _l2_trigger_count_1min = 0;
    }

    uint32_t l1_since_ms() const { return _l1_since_ms; }

    bool l1_active_long_enough(uint32_t now_ms, uint32_t threshold_ms) const
    {
        return _l1_warning &&
               _l1_since_ms != 0 &&
               (now_ms - _l1_since_ms) >= threshold_ms;
    }

    void evaluate_levels(uint32_t now_ms)
    {
        bool new_l1 = false;
        bool new_l2 = false;
        bool new_l3 = false;
        uint8_t reason = 0;

        // reason code:
        // 1 = low confidence
        // 2 = target lost
        // 3 = high reproj error
        // 4 = high covariance
        // 5 = repeated L2 within 1 minute

        // L2 conditions first
        if (_target_lost) {
            new_l2 = true;
            reason = 2;
        } else if (_status_confidence < 0.50f) {
            new_l2 = true;
            reason = 1;
        } else if (_reproj_error > 0.30f) {
            new_l2 = true;
            reason = 3;
        } else if (_covariance > 0.20f) {
            new_l2 = true;
            reason = 4;
        }

        // L1 conditions
        if (!new_l2) {
            if (_status_confidence < 0.70f) {
                new_l1 = true;
                reason = 1;
            } else if (_reproj_error > 0.10f) {
                new_l1 = true;
                reason = 3;
            } else if (_covariance > 0.05f) {
                new_l1 = true;
                reason = 4;
            }
        }

        // track L1 duration
        if (new_l1) {
            if (_l1_since_ms == 0) {
                _l1_since_ms = now_ms;
            }
        } else {
            _l1_since_ms = 0;
        }

        // track repeated L2 within 1 minute for L3
        if (new_l2 && !_l2_go_around) {
            if (_l2_last_trigger_ms == 0 || (now_ms - _l2_last_trigger_ms) > 60000U) {
                _l2_trigger_count_1min = 1;
            } else {
                _l2_trigger_count_1min++;
            }
            _l2_last_trigger_ms = now_ms;
        }

        if (new_l2 && _l2_trigger_count_1min >= 2) {
            new_l3 = true;
            reason = 5;
        }

        _l1_warning = new_l1;
        _l2_go_around = new_l2;
        _l3_takeover = new_l3;
        _level_reason = reason;
    }

    void update_correction(float roll, float pitch, float yaw,
                           float x, float y, float z,
                           float dist, float conf,
                           uint8_t flags,
                           uint32_t now_ms)
    {
        _roll_err = roll;
        _pitch_err = pitch;

        _corr_flags = flags;
        _yaw_valid = (flags & 0x01U) != 0;       // bit0: yaw_valid
        _distance_valid = (flags & 0x02U) != 0;  // bit1: distance_valid

        // 只有有效時才採用 yaw / distance
        if (_yaw_valid) {
            _yaw_err = yaw;
        } else {
            _yaw_err = 0.0f;
        }

        _x_err = x;
        _y_err = y;
        _z_err = z;

        // Story 2.7: camera mount offset compensation
        _x_err_corr = _x_err + _cam_offset_x;
        _y_err_corr = _y_err + _cam_offset_y;
        _z_err_corr = _z_err + _cam_offset_z;

        if (_distance_valid) {
            _distance = dist;
        } else {
            _distance = 0.0f;
        }

        _corr_confidence = conf;
        _last_corr_update_ms = now_ms;
        _have_corr = true;
    }

    void set_camera_offset(float x, float y, float z)
    {
        _cam_offset_x = x;
        _cam_offset_y = y;
        _cam_offset_z = z;
    }

    void update_status(float conf, bool lost,
                   float reproj, float cov,
                   uint32_t now_ms)
    {
        _status_confidence = conf;
        _target_lost = lost;

        _reproj_error = reproj;
        _covariance = cov;

        _last_status_update_ms = now_ms;
        _have_status = true;
    }

    float roll_err() const { return _roll_err; }
    float pitch_err() const { return _pitch_err; }
    float yaw_err() const { return _yaw_err; }

    float x_err() const { return _x_err; }
    float y_err() const { return _y_err; }
    float z_err() const { return _z_err; }

    float x_err_corr() const { return _x_err_corr; }
    float y_err_corr() const { return _y_err_corr; }
    float z_err_corr() const { return _z_err_corr; }

    float distance() const { return _distance; }
    float confidence() const { return _status_confidence; }

    float correction_confidence() const { return _corr_confidence; }

    bool target_lost() const { return _target_lost; }

    uint32_t last_corr_update_ms() const { return _last_corr_update_ms; }
    uint32_t last_status_update_ms() const { return _last_status_update_ms; }

    bool correction_recent(uint32_t now_ms, uint32_t timeout_ms) const
    {
        return _last_corr_update_ms != 0 &&
               (now_ms - _last_corr_update_ms) <= timeout_ms;
    }

    bool correction_valid(uint32_t now_ms, uint32_t timeout_ms, float min_conf) const
    {
        return correction_recent(now_ms, timeout_ms) &&
               _corr_confidence >= min_conf;
    }

    bool yaw_valid() const { return _yaw_valid; }
    bool distance_valid() const { return _distance_valid; }
    uint8_t correction_flags() const { return _corr_flags; }

    // ===== AI LINK (Story 2.4) =====
    void update_link_health(uint32_t now_ms)
    {
        bool corr_to = _have_corr &&
               ((now_ms - _last_corr_update_ms) > 1000);

        bool status_to = _have_status &&
                 ((now_ms - _last_status_update_ms) > 1000);

        _corr_timeout = corr_to;
        _status_timeout = status_to;

        bool lost_now = corr_to || status_to;

        if (lost_now && !_ai_link_lost) {
            _ai_link_lost = true;
            _timeout_start_ms = now_ms;

            if (corr_to && status_to) _ai_error_code = 3003;
            else if (corr_to) _ai_error_code = 3001;
            else _ai_error_code = 3002;
        }

        if (!lost_now && _ai_link_lost) {
            _ai_link_lost = false;
            _last_recover_ms = now_ms;
            _recovery_pending = true;
            _ai_error_code = 0;
        }
    }

    bool ai_link_lost() const { return _ai_link_lost; }

    uint16_t ai_error_code() const { return _ai_error_code; }

    uint32_t ai_timeout_duration_ms(uint32_t now_ms) const
    {
        if (!_ai_link_lost || _timeout_start_ms == 0) return 0;
        return now_ms - _timeout_start_ms;
    }

    bool correction_allowed(uint32_t now_ms) const
    {
        return !_ai_link_lost &&
            _last_corr_update_ms != 0 &&
            (now_ms - _last_corr_update_ms <= 1000);
    }

    bool recovery_pending() const { return _recovery_pending; }

    void clear_recovery_pending() { _recovery_pending = false; }

    // reset per-landing-attempt display/action state, but keep repeated-L2 history
    void reset_landing_cycle_state()
    {
        _l1_warning = false;
        _l2_go_around = false;
        _l3_takeover = false;
        _level_reason = 0;
        _l1_since_ms = 0;
    }

    void reset_auto_stream_flags()
    {
        _auto_start_sent = false;
        _auto_stop_sent = false;
        _auto_landing_armed_seen = false;
    }

    bool auto_start_sent() const { return _auto_start_sent; }
    bool auto_stop_sent() const { return _auto_stop_sent; }

    void mark_auto_start_sent() { _auto_start_sent = true; }
    void mark_auto_stop_sent() { _auto_stop_sent = true; }

    bool auto_landing_armed_seen() const { return _auto_landing_armed_seen; }
    void mark_auto_landing_armed_seen() { _auto_landing_armed_seen = true; }

    bool ai_vision_enabled() const { return _ai_vision_enabled; }
    void set_ai_vision_enabled(bool enabled) { _ai_vision_enabled = enabled; }

    void vision_ack_start_wait(uint16_t command, uint32_t now_ms)
    {
        _vision_ack_pending = true;
        _vision_ack_command = command;
        _vision_ack_sent_ms = now_ms;
        _vision_last_result = 255;          // invalid / not received yet
        _vision_last_result_param2 = 0;
        _vision_last_success = false;
        _vision_timeout_reported = false;
    }

    bool vision_ack_pending() const { return _vision_ack_pending; }
    uint16_t vision_ack_command() const { return _vision_ack_command; }
    uint8_t vision_last_result() const { return _vision_last_result; }
    uint32_t vision_last_result_param2() const { return _vision_last_result_param2; }
    bool vision_last_success() const { return _vision_last_success; }

    bool vision_ack_timed_out(uint32_t now_ms, uint32_t timeout_ms) const
    {
        return _vision_ack_pending &&
               _vision_ack_sent_ms != 0 &&
               (now_ms - _vision_ack_sent_ms) > timeout_ms;
    }

    void vision_ack_mark_timeout()
    {
        _vision_ack_pending = false;
        _vision_last_result = 255;   // timeout sentinel
        _vision_last_result_param2 = 0;
        _vision_last_success = false;
        _vision_timeout_reported = true;
    }

    bool vision_timeout_reported() const { return _vision_timeout_reported; }

    bool vision_ack_handle(uint16_t command, uint8_t result, uint32_t result_param2)
    {
        if (!_vision_ack_pending) {
            return false;
        }
        if (_vision_ack_command != command) {
            return false;
        }

        _vision_ack_pending = false;
        _vision_last_result = result;
        _vision_last_result_param2 = result_param2;
        _vision_last_success = (result == 0); // MAV_RESULT_ACCEPTED = 0
        _vision_last_success = (result == MAV_RESULT_ACCEPTED);
        _vision_timeout_reported = false;
        return true;
    }

private:

    bool _l1_warning = false;
    bool _l2_go_around = false;
    bool _l3_takeover = false;

    uint8_t _level_reason = 0;

    uint32_t _l1_since_ms = 0;
    uint32_t _l2_last_trigger_ms = 0;
    uint8_t _l2_trigger_count_1min = 0;

    float _roll_err = 0;
    float _pitch_err = 0;
    float _yaw_err = 0;

    float _x_err = 0;
    float _y_err = 0;
    float _z_err = 0;

    float _x_err_corr = 0.0f;
    float _y_err_corr = 0.0f;
    float _z_err_corr = 0.0f;

    float _cam_offset_x = 0.0f;
    float _cam_offset_y = 0.0f;
    float _cam_offset_z = 0.0f;

    float _distance = 0;

    uint32_t _last_corr_update_ms = 0;
    bool _have_corr = false;

    float _corr_confidence = 0;
    float _status_confidence = 0;

    bool _target_lost = false;

    float _reproj_error = 0;
    float _covariance = 0;

    bool _yaw_valid = false;
    bool _distance_valid = false;
    uint8_t _corr_flags = 0;

    // ===== AI LINK DIAGNOSTICS (Story 2.4) =====

    uint32_t _last_status_update_ms = 0;
    bool _have_status = false;

    bool _corr_timeout = false;
    bool _status_timeout = false;
    bool _ai_link_lost = false;

    uint32_t _timeout_start_ms = 0;
    uint32_t _last_recover_ms = 0;

    bool _recovery_pending = false;

    bool _auto_start_sent = false;
    bool _auto_stop_sent = false;
    bool _auto_landing_armed_seen = false;

    uint16_t _ai_error_code = 0;
    // ===== Story 3.2 AI VISION =====
    bool _ai_vision_enabled = false;

        // ===== Story 3.2 AI VISION ACK TRACKING =====
    bool _vision_ack_pending = false;
    uint16_t _vision_ack_command = 0;
    uint32_t _vision_ack_sent_ms = 0;

    uint8_t _vision_last_result = 255; // 255 = not received / timeout sentinel
    uint32_t _vision_last_result_param2 = 0;
    bool _vision_last_success = false;
    bool _vision_timeout_reported = false;
};