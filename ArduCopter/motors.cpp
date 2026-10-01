#include "Copter.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

static uint32_t auto_disarm_begin;

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
void Copter::arm_motors_check()
{
    static int16_t arming_counter;

    // the KFT two-stick (DJI-style) gesture completely replaces rudder
    // arming when enabled, so that the two cannot stack on the same
    // stick positions.  Note this also disables stick-triggered AutoTrim,
    // which shares the rudder-arm gesture.
    if (g2.kft_arm_en) {
        arming_counter = 0;
        kft_stick_arming_check();
        return;
    }

    // check if arming/disarm using rudder is allowed
    AP_Arming::RudderArming arming_rudder = arming.get_rudder_arming_type();
    if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
        arming_counter = 0;
        return;
    }

#if TOY_MODE_ENABLED
    if (g2.toy_mode.enabled()) {
        // not armed with sticks in toy mode
        return;
    }
#endif

    // ensure throttle is down
    if (channel_throttle->get_control_in() > 0) {
        arming_counter = 0;
        return;
    }

    int16_t yaw_in = channel_yaw->get_control_in();

    // full right
    if (yaw_in > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if (arming_counter <= AUTO_TRIM_DELAY) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors->armed()) {
            // reset arming counter if arming fail
            if (!arming.arm(AP_Arming::Method::RUDDER)) {
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors->armed() && flightmode->mode_number() == Mode::Number::STABILIZE) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim start");
            auto_trim_counter = 250;
            auto_trim_started = false;
            // ensure auto-disarm doesn't trigger immediately
            auto_disarm_begin = millis();
        }

    // full left and rudder disarming is enabled
    } else if ((yaw_in < -4000) && (arming_rudder == AP_Arming::RudderArming::ARMDISARM)) {
        if (!flightmode->has_manual_throttle() && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if (arming_counter <= DISARM_DELAY) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors->armed()) {
            arming.disarm(AP_Arming::Method::RUDDER);
        }

    // Yaw is centered so reset arming counter
    } else {
        arming_counter = 0;
    }
}

// ============================================================================
//  KFT | DJI-style two-stick arming
// ----------------------------------------------------------------------------
//  Native C++ port of dji_stick_arming.lua v1.5.0 (KFT).
//
//  Reads raw RC PWM on Roll(1)/Pitch(2)/Throttle(3)/Yaw(4) and arms or disarms
//  when a two-stick gesture is held continuously.  Mode 2 transmitter:
//
//    ARM    : Throttle LOW, Yaw HIGH, Roll LOW,  Pitch HIGH  held KFT_ARM_MS
//    DISARM : Throttle LOW, Yaw LOW,  Roll HIGH, Pitch HIGH  held KFT_ARM_DIS_MS
//
//  "LOW" means raw PWM below KFT_ARM_LOW, "HIGH" means above KFT_ARM_HIGH
//  (defaults 1280/1720 for a Min=1051 Mid=1501 Max=1951 calibration).
//  Breaking the gesture at any point resets the hold timer.  Arming is
//  additionally gated on throttle still being LOW at the instant the hold
//  completes, on top of the normal pre-arm/arm check pipeline which is
//  entered through arming.arm() exactly as rudder arming does.
//
//  Enabled with KFT_ARM_EN=1, which also takes over from rudder arming; set
//  ARMING_RUDDER=0 to silence the startup warning.
//
//  called from arm_motors_check() at 10Hz
// ============================================================================

// send a throttled "ARMING... 43%" style progress message, returns the
// millis() to store as the new last-notification time
uint32_t Copter::kft_stick_notify(const char *label, uint32_t elapsed_ms, uint32_t total_ms, uint32_t last_notify_ms)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_notify_ms < 500) {
        return last_notify_ms;
    }
    const uint8_t pct = (total_ms > 0) ? MIN((uint32_t)100, (elapsed_ms * 100) / total_ms) : 0;
    gcs().send_text(MAV_SEVERITY_INFO, "KFT | %s... %u%%", label, (unsigned)pct);
    return now_ms;
}

// arm gesture: throttle LOW, yaw HIGH, roll LOW, pitch HIGH
bool Copter::kft_stick_arm_gesture() const
{
    const uint16_t low = g2.kft_arm_low;
    const uint16_t high = g2.kft_arm_high;
    return channel_throttle->get_radio_in() < low &&
           channel_yaw->get_radio_in()      > high &&
           channel_roll->get_radio_in()     < low &&
           channel_pitch->get_radio_in()    > high;
}

// disarm gesture: throttle LOW, yaw LOW, roll HIGH, pitch HIGH
bool Copter::kft_stick_disarm_gesture() const
{
    const uint16_t low = g2.kft_arm_low;
    const uint16_t high = g2.kft_arm_high;
    return channel_throttle->get_radio_in() < low &&
           channel_yaw->get_radio_in()      < low &&
           channel_roll->get_radio_in()     > high &&
           channel_pitch->get_radio_in()    > high;
}

// kft_stick_arming_check - arm or disarm on a held two-stick gesture
// called at 10Hz from arm_motors_check() when KFT_ARM_EN is set
void Copter::kft_stick_arming_check()
{
    const uint32_t now_ms = AP_HAL::millis();

    // warn once if the built-in rudder arming is still configured.  We do not
    // write the parameter ourselves: silently changing an arming parameter
    // behind the pilot's back is worse than a warning, and the rudder-arm path
    // is skipped in code anyway so the two can never stack.
    if (!kft_stick_arm.rudder_warned &&
        arming.get_rudder_arming_type() != AP_Arming::RudderArming::IS_DISABLED) {
        kft_stick_arm.rudder_warned = true;
        gcs().send_text(MAV_SEVERITY_WARNING, "KFT | stick arming active, set ARMING_RUDDER=0");
    }

    // do not act on stick positions we cannot trust
    if (!rc().has_valid_input()) {
        kft_stick_arm.holding_arm = false;
        kft_stick_arm.holding_disarm = false;
        return;
    }

    // ---- ARM ----------------------------------------------------------
    if (kft_stick_arm_gesture()) {
        if (!kft_stick_arm.holding_arm) {
            kft_stick_arm.arm_start_ms = now_ms;
            kft_stick_arm.arm_notify_ms = now_ms;
            kft_stick_arm.holding_arm = true;
            kft_stick_arm.holding_disarm = false;
        } else {
            const uint32_t elapsed_ms = now_ms - kft_stick_arm.arm_start_ms;
            if (elapsed_ms >= (uint32_t)g2.kft_arm_ms) {
                if (!motors->armed()) {
                    // extra local guard on top of the standard arming checks
                    if (channel_throttle->get_radio_in() < (uint16_t)g2.kft_arm_low) {
                        if (arming.arm(AP_Arming::Method::RUDDER)) {
                            gcs().send_text(MAV_SEVERITY_INFO, "KFT | *** ARMED ***");
                        } else {
                            gcs().send_text(MAV_SEVERITY_WARNING, "KFT | arm failed, fix pre-arm errors");
                        }
                    } else {
                        gcs().send_text(MAV_SEVERITY_WARNING, "KFT | arm blocked, throttle not low");
                    }
                }
                kft_stick_arm.holding_arm = false;
            } else {
                kft_stick_arm.arm_notify_ms = kft_stick_notify("ARMING", elapsed_ms, g2.kft_arm_ms, kft_stick_arm.arm_notify_ms);
            }
        }
    } else {
        kft_stick_arm.holding_arm = false;
    }

    // ---- DISARM -------------------------------------------------------
    if (kft_stick_disarm_gesture()) {
        if (!kft_stick_arm.holding_disarm) {
            kft_stick_arm.disarm_start_ms = now_ms;
            kft_stick_arm.disarm_notify_ms = now_ms;
            kft_stick_arm.holding_disarm = true;
            kft_stick_arm.holding_arm = false;
        } else {
            const uint32_t elapsed_ms = now_ms - kft_stick_arm.disarm_start_ms;
            if (elapsed_ms >= (uint32_t)g2.kft_arm_dis_ms) {
                if (motors->armed()) {
                    if (arming.disarm(AP_Arming::Method::RUDDER)) {
                        gcs().send_text(MAV_SEVERITY_INFO, "KFT | *** DISARMED ***");
                    } else {
                        gcs().send_text(MAV_SEVERITY_WARNING, "KFT | disarm failed");
                    }
                }
                kft_stick_arm.holding_disarm = false;
            } else {
                kft_stick_arm.disarm_notify_ms = kft_stick_notify("DISARMING", elapsed_ms, g2.kft_arm_dis_ms, kft_stick_arm.disarm_notify_ms);
            }
        }
    } else {
        kft_stick_arm.holding_disarm = false;
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
void Copter::auto_disarm_check()
{
    uint32_t tnow_ms = millis();
    uint32_t disarm_delay_ms = 1000*constrain_int16(g.disarm_delay, 0, 127);

    // exit immediately if we are already disarmed, or if auto
    // disarming is disabled
    if (!motors->armed() || disarm_delay_ms == 0 || flightmode->mode_number() == Mode::Number::THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // if the rotor is still spinning, don't initiate auto disarm
    if (motors->get_spool_state() > AP_Motors::SpoolState::GROUND_IDLE) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // always allow auto disarm if using interlock switch or motors are Emergency Stopped
    if ((ap.using_interlock && !motors->get_interlock()) || SRV_Channels::get_emergency_stop()) {
#if FRAME_CONFIG != HELI_FRAME
        // use a shorter delay if using throttle interlock switch or Emergency Stop, because it is less
        // obvious the copter is armed as the motors will not be spinning
        disarm_delay_ms /= 2;
#endif
    } else {
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (flightmode->has_manual_throttle() || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = get_throttle_mid() + g.throttle_deadzone;
            thr_low = channel_throttle->get_control_in() <= deadband_top;
        }

        if (!thr_low || !ap.land_complete) {
            // reset timer
            auto_disarm_begin = tnow_ms;
        }
    }

    // disarm once timer expires
    if ((tnow_ms-auto_disarm_begin) >= disarm_delay_ms) {
        arming.disarm(AP_Arming::Method::DISARMDELAY);
        auto_disarm_begin = tnow_ms;
    }
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Copter::motors_output()
{
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // this is to allow the failsafe module to deliberately crash
    // the vehicle. Only used in extreme circumstances to meet the
    // OBC rules
    if (g2.afs.should_crash_vehicle()) {
        g2.afs.terminate_vehicle();
        if (!g2.afs.terminating_vehicle_via_landing()) {
            return;
        }
        // landing must continue to run the motors output
    }
#endif

    // Update arming delay state
    if (ap.in_arming_delay && (!motors->armed() || millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f || flightmode->mode_number() == Mode::Number::THROW)) {
        ap.in_arming_delay = false;
    }

    // output any servo channels
    SRV_Channels::calc_pwm();

    auto &srv = AP::srv();

    // cork now, so that all channel outputs happen at once
    srv.cork();

    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();

    // update motors interlock state
    bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !SRV_Channels::get_emergency_stop();
    if (!motors->get_interlock() && interlock) {
        motors->set_interlock(true);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_ENABLED);
    } else if (motors->get_interlock() && !interlock) {
        motors->set_interlock(false);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_DISABLED);
    }

    if (ap.motor_test) {
        // check if we are performing the motor test
        motor_test_output();
    } else {
        // send output signals to motors
        flightmode->output_to_motors();
    }

    // push all channels
    srv.push();
}

// check for pilot stick input to trigger lost vehicle alarm
void Copter::lost_vehicle_check()
{
    static uint8_t soundalarm_counter;

    // disable if aux switch is setup to vehicle alarm as the two could interfere
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::LOST_VEHICLE_SOUND)) {
        return;
    }

    // ensure throttle is down, motors not armed, pitch and roll rc at max. Note: rc1=roll rc2=pitch
    if (ap.throttle_zero && !motors->armed() && (channel_roll->get_control_in() > 4000) && (channel_pitch->get_control_in() > 4000)) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs().send_text(MAV_SEVERITY_NOTICE,"Locate Copter alarm");
            }
        } else {
            soundalarm_counter++;
        }
    } else {
        soundalarm_counter = 0;
        if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;
        }
    }
}
