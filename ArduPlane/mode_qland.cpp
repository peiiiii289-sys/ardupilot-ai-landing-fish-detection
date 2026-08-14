#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQLand::_enter()
{
    plane.mode_qloiter._enter();
    quadplane.throttle_wait = false;
    quadplane.setup_target_position();
    poscontrol.set_state(QuadPlane::QPOS_LAND_DESCEND);
    // Story 2.5: 每次新的 QLAND 流程都重置一次性 Start/Stop 旗標
    plane.landing_ai.reset_auto_stream_flags();
    // Epic4 fix: reset landing cycle state (for repeated L2 → L3)
    plane.landing_ai.reset_landing_cycle_state();

    
    quadplane.last_land_final_agl_m = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
    quadplane.landing_detect.lower_limit_start_ms = 0;
    quadplane.landing_detect.land_start_ms = 0;
#if AP_LANDINGGEAR_ENABLED
    plane.g2.landing_gear.deploy_for_landing();
#endif

#if HAL_MOUNT_ENABLED
    if (plane.g.ail_land_gimbal.get()) {
        plane.camera_mount.set_angle_target(0.0f, -90.0f, 0.0f, false);
        gcs().send_text(MAV_SEVERITY_INFO, "AI_GIMBAL_AUTO pitch=-90 QLAND");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "AI_GIMBAL_SKIP disabled");
    }
#endif
    return true;
}

void ModeQLand::update()
{
    plane.mode_qstabilize.update();
}


void ModeQLand::run()
{
    static uint32_t last_ai_log_ms = 0;
    static uint32_t last_ai_dbg_ms = 0;
    const uint32_t now = AP_HAL::millis();

    plane.landing_ai.update_link_health(now);
    
    // Story 2.5: shared AGL/Home fallback auto-start
    plane.try_ai_landing_auto_start("QLAND");

    if (now - last_ai_dbg_ms > 1000) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "AI_LINK_DBG lost=%u err=%u corr_age=%lu status_age=%lu",
                        (unsigned)plane.landing_ai.ai_link_lost(),
                        (unsigned)plane.landing_ai.ai_error_code(),
                        (unsigned long)(plane.landing_ai.last_corr_update_ms() == 0 ? 0 : now - plane.landing_ai.last_corr_update_ms()),
                        (unsigned long)(plane.landing_ai.last_status_update_ms() == 0 ? 0 : now - plane.landing_ai.last_status_update_ms()));
        last_ai_dbg_ms = now;
    }

    // timeout -> safety action
    if (plane.landing_ai.ai_link_lost()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,
                        "AI_LINK_TIMEOUT err=%u dur=%lu",
                        (unsigned)plane.landing_ai.ai_error_code(),
                        (unsigned long)plane.landing_ai.ai_timeout_duration_ms(now));

        plane.set_mode(plane.mode_qhover, ModeReason::SCRIPTING);
        return;
    }

    // recovery message
    if (plane.landing_ai.recovery_pending()) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "AI_LINK_RECOVERED");
        plane.landing_ai.clear_recovery_pending();
    }

    const bool ai_ok = !plane.landing_ai.target_lost() &&
                    plane.landing_ai.confidence() > 0.5f;

    float ai_pitch_rad = 0.0f;
    if (plane.landing_ai.correction_allowed(now)) {
        ai_pitch_rad =
            constrain_float(plane.landing_ai.pitch_err(), -0.10f, 0.10f);
    }

    const float ai_pitch_cd = degrees(ai_pitch_rad) * 100.0f;
    const float ai_pitch_inject_cd = ai_pitch_cd * 0.3f;

    if (now - last_ai_log_ms > 1000) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "QLAND AI pitch=%.3f conf=%.2f lost=%u inject=%.1f ok=%u",
                        (double)plane.landing_ai.pitch_err(),
                        (double)plane.landing_ai.confidence(),
                        (unsigned)plane.landing_ai.target_lost(),
                        (double)ai_pitch_inject_cd,
                        (unsigned)ai_ok);
        gcs().send_text(MAV_SEVERITY_INFO,
                        "AI_CAM_OFF raw=(%.2f,%.2f,%.2f) corr=(%.2f,%.2f,%.2f)",
                        (double)plane.landing_ai.x_err(),
                        (double)plane.landing_ai.y_err(),
                        (double)plane.landing_ai.z_err(),
                        (double)plane.landing_ai.x_err_corr(),
                        (double)plane.landing_ai.y_err_corr(),
                        (double)plane.landing_ai.z_err_corr());
        gcs().send_text(MAV_SEVERITY_INFO,
                        "AI_CTRL_USE xyz=(%.2f,%.2f,%.2f)",
                        (double)plane.landing_ai.x_err_corr(),
                        (double)plane.landing_ai.y_err_corr(),
                        (double)plane.landing_ai.z_err_corr());
        last_ai_log_ms = now;
    }

    static bool l2_abort_triggered = false;

    if (plane.landing_ai.l2_go_around()) {
        if (!l2_abort_triggered) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,
                            "AI_LANDING_ABORT_L2 reason=%u",
                            (unsigned)plane.landing_ai.level_reason());

            // ===== Go-Around (Full Spec Version) =====
            float current_alt = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
            // H_safe = max(current + 5m, 15m)
            float h_safe = MAX(current_alt + 5.0f, 15.0f);
            // 設定目標高度（讓飛機先爬升）
            plane.next_WP_loc.alt = h_safe * 100.0f; // cm

            const float go_around_radius_m = 15.0f;
            const float go_around_speed_mps = 4.0f;

            // ===== 接入速度控制 =====
            gcs().send_text(MAV_SEVERITY_INFO,
                            "AI_GO_AROUND target_alt=%.1fm radius=%.1fm speed=%.1fmps",
                            (double)h_safe,
                            (double)go_around_radius_m,
                            (double)go_around_speed_mps);
            // 切到 QLOITER（盤旋）
            plane.set_mode(plane.mode_qloiter, ModeReason::SCRIPTING);
            l2_abort_triggered = true;
            return;
        }
    } else {
        l2_abort_triggered = false;
    }
    
    plane.mode_qloiter.run();

    if (plane.arming.is_armed()) {
        plane.landing_ai.mark_auto_landing_armed_seen();
    }

    // Story 2.5: 正式版 STOP
    // 在目前這個 QLAND 執行路徑中，landing 完成後會先出現 Land complete / Throttle disarmed，
    // 因此以「本次 QLAND 已經送過 START，且目前已 disarm」作為 STOP 觸發條件。
    if (plane.landing_ai.auto_start_sent() &&
        !plane.landing_ai.auto_stop_sent() &&
        plane.landing_ai.auto_landing_armed_seen() &&
        !plane.arming.is_armed()) {

        plane.send_ai_landing_stop_auto();
        gcs().send_text(MAV_SEVERITY_INFO, "AI_AUTO_STOP trigger source=QLAND_DISARM");
        plane.landing_ai.mark_auto_stop_sent();
    }

}

#endif
