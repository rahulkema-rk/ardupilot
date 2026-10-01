#include "Copter.h"

#if MODE_LOITER_ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeLoiter::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

#if AC_PRECLAND_ENABLED
    _precision_loiter_active = false;
#endif

    return true;
}

#if AC_PRECLAND_ENABLED
bool ModeLoiter::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void ModeLoiter::precision_loiter_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel;
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos = inertial_nav.get_position_xy_cm();
    }
    // get the velocity of the target
    copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

    Vector2f zero;
    Vector2p landing_pos = target_pos.topostype();
    // target vel will remain zero if landing target is stationary
    pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    // run pos controller
    pos_control->update_xy_controller();
}
#endif

/*
 * L-Turn (LTRN)
 *
 * A sharp, axis-decoupled corner flown inside Loiter.  Stock Loiter cannot produce
 * one because AC_Loiter::calc_desired_velocity() bleeds off the existing velocity
 * through a velocity-proportional drag term with a multi-second time constant; the
 * hard brake is gated behind LOIT_BRK_DELAY with the sticks centred and so never
 * fires during a pitch-to-roll stick sweep.  The result is a rounded corner.
 *
 * While the state machine is active we bypass loiter_nav for the horizontal axes
 * and drive AC_PosControl directly with a body-frame velocity target plus an
 * explicit acceleration feed forward, under temporarily raised speed/accel/jerk
 * limits.  The vertical axis is untouched in every state, and when the state is
 * INACTIVE ModeLoiter::run() takes its original code path unchanged.
 *
 * The acceleration command is clamped as a VECTOR, not per axis.  AC_PosControl's
 * accel_to_lean_angles() constrains roll and pitch independently, so a simultaneous
 * full-ceiling demand on both body axes yields up to sqrt(2) x ANGLE_MAX of total
 * lean.  LTRN_XPRIO decides how the budget is split when the demand saturates.
 */

// body-X speed below which the vehicle counts as stopped for the crossover test, m/s
#define LTRN_XOVER_MIN_MS   0.20f

// floors that keep a misconfigured parameter from handing the position controller a
// zero acceleration or jerk limit
#define LTRN_ACCEL_MIN_MSS  0.5f
#define LTRN_JERK_MIN_MSSS  1.0f

// EXITING handover gate.  Loiter inherits PosControl's desired velocity and overwrites
// its desired acceleration, so the stop only releases once both already match what
// Loiter will continue with.  A mismatch that persists for LTRN_HANDOVER_TIMEOUT_MS
// after speed reaches LTRN_EXIT_V releases anyway, i.e. degrades to the old release.
#define LTRN_HANDOVER_VEL_TOL_MS     0.25f
#define LTRN_HANDOVER_ACC_TOL_MSS    0.50f
#define LTRN_HANDOVER_TIMEOUT_MS     1000U
// measured attitude lag (log_0078/79: 100-170 ms); widens the velocity tolerance while
// accelerating, where the aircraft trails its desired velocity by roughly accel x lag
#define LTRN_HANDOVER_LAG_S          0.15f

// exit blend.  A held stick is any pilot acceleration demand above this; the blend may
// extend the handover timeout by the time its remaining slew needs, up to this cap.
#define LTRN_BLEND_STICK_MIN_MSS     0.20f
#define LTRN_BLEND_EXTRA_MAX_MS      2000.0f

// trigger arming.  The pitch stick must have exceeded LTRN_PIT_ARM within this window
// for a roll input to start a turn: an L-Turn is a corner out of commanded forward
// flight, not a response to a roll stick given while the vehicle merely has speed.
#define LTRN_PIT_ARM_MS              1500U
// re-trigger out of EXITING.  The roll input must oppose the current lateral velocity
// for this long - a deliberate reversal, not the pilot correcting an overrun.
#define LTRN_RETRIG_MS               300U

// human readable name for each exit path; index must track LTurnExit
const char *ModeLoiter::ltrn_exit_name(LTurnExit reason)
{
    switch (reason) {
    case LTurnExit::NONE:          return "none";
    case LTurnExit::STICK_CENTRED: return "stick centred";
    case LTurnExit::RC_LOSS:       return "RC failsafe";
    case LTurnExit::EKF_FAILSAFE:  return "EKF failsafe";
    case LTurnExit::BATT_FAILSAFE: return "battery failsafe";
    case LTurnExit::LANDED:        return "landed/disarmed";
    case LTurnExit::NOT_FLYING:    return "not flying";
    case LTurnExit::MODE_CHANGE:   return "mode change";
    case LTurnExit::STOPPED:       return "stopped";
    case LTurnExit::EXIT_TIMEOUT:  return "exit timeout";
    case LTurnExit::EXIT_DISTANCE: return "exit distance";
    case LTurnExit::HANDOVER_TIMEOUT: return "handover timeout";
    case LTurnExit::STICK_BLEND:   return "stick blend";
    }
    return "unknown";
}

// The single accessor for pilot roll input.  The trigger test, the exit test and the
// LTRN log all call this, so the value written to the log is provably the value that
// was compared - the two can never drift apart again.
float ModeLoiter::ltrn_roll_input() const
{
    if (copter.failsafe.radio || !rc().has_ever_seen_rc_input()) {
        return 0.0f;
    }
    return channel_roll->get_control_in() * (1.0f / ROLL_PITCH_YAW_INPUT_MAX);
}

float ModeLoiter::ltrn_pitch_input() const
{
    if (copter.failsafe.radio || !rc().has_ever_seen_rc_input()) {
        return 0.0f;
    }
    return channel_pitch->get_control_in() * (1.0f / ROLL_PITCH_YAW_INPUT_MAX);
}

// returns the vehicle's horizontal velocity in the latched heading's body frame, m/s
Vector2f ModeLoiter::ltrn_body_velocity_ms() const
{
    return ltrn_ne_to_body(inertial_nav.get_velocity_xy_cms() * 0.01f);
}

// rotates an NE vector into the latched heading's body frame
Vector2f ModeLoiter::ltrn_ne_to_body(const Vector2f &ne) const
{
    return Vector2f{ ne.x * _ltrn_cos_yaw + ne.y * _ltrn_sin_yaw,
                    -ne.x * _ltrn_sin_yaw + ne.y * _ltrn_cos_yaw };
}

// clamp LTRN_DECEL and LTRN_LAT_ACC to the acceleration the airframe can actually
// produce inside its lean angle envelope, warning once per boot if we had to
void ModeLoiter::ltrn_update_accel_limits()
{
    // ANGLE_MAX, further limited by the position controller's own tilt envelope
    float angle_max_cd = MIN(attitude_control->lean_angle_max_cd(), pos_control->get_lean_angle_max_cd());

    // LOIT_ANG_MAX overrides ANGLE_MAX for pilot commanded lean in loiter when non-zero.
    // AC_Loiter only exposes get_angle_max_cd(), which folds in a 2/3 softening factor
    // we do not want here, so the raw parameter is read directly.
    if (_ltrn_loit_ang_max != nullptr && is_positive(_ltrn_loit_ang_max->get())) {
        angle_max_cd = MIN(angle_max_cd, _ltrn_loit_ang_max->get() * 100.0f);
    }

    // thrust based limit: a heavy airframe at high hover throttle cannot reach
    // ANGLE_MAX without losing altitude, and accel_to_lean_angles() enforces this
    // downstream of us.  Folding it in here keeps the logged demand honest.
    angle_max_cd = MIN(angle_max_cd, attitude_control->get_althold_lean_angle_max_cd());

    const float margin = constrain_float(g2.ltrn_margin, 0.70f, 1.0f);

    // the floor keeps a misconfigured ANGLE_MAX from handing the position controller
    // a zero acceleration limit, which trips an internal error in shape_vel_accel_xy
    _ltrn_ceiling_ms = MAX(GRAVITY_MSS * tanf(radians(angle_max_cd * 0.01f)) * margin,
                           LTRN_ACCEL_MIN_MSS);

    _ltrn_decel_ms = constrain_float(g2.ltrn_decel, LTRN_ACCEL_MIN_MSS, _ltrn_ceiling_ms);
    _ltrn_lat_acc_ms = constrain_float(g2.ltrn_lat_acc, LTRN_ACCEL_MIN_MSS, _ltrn_ceiling_ms);

    // the clamp itself is silent in the fast loop; warn once per boot so the logs stay readable
    if (!_ltrn_clamp_warned && (g2.ltrn_decel > _ltrn_ceiling_ms || g2.ltrn_lat_acc > _ltrn_ceiling_ms)) {
        _ltrn_clamp_warned = true;
        if (g2.ltrn_decel > _ltrn_ceiling_ms) {
            gcs().send_text(MAV_SEVERITY_WARNING, "LTRN: LTRN_DECEL clamped to %.1f m/s/s", (double)_ltrn_decel_ms);
        }
        if (g2.ltrn_lat_acc > _ltrn_ceiling_ms) {
            gcs().send_text(MAV_SEVERITY_WARNING, "LTRN: LTRN_LAT_ACC clamped to %.1f m/s/s", (double)_ltrn_lat_acc_ms);
        }
    }
}

// raise the position controller's horizontal jerk limit for the duration of the manoeuvre.
// PSC_JERK_XY has no setter on AC_PosControl, so the parameter itself is overridden in
// memory (never saved) and restored on exit.
void ModeLoiter::ltrn_set_jerk(float jerk_msss)
{
    if (_ltrn_psc_jerk == nullptr) {
        return;
    }
    if (!_ltrn_jerk_overridden) {
        _ltrn_saved_jerk_msss = _ltrn_psc_jerk->get();
        _ltrn_jerk_overridden = true;
    }
    _ltrn_psc_jerk->set(MAX(jerk_msss, LTRN_JERK_MIN_MSSS));
}

void ModeLoiter::ltrn_restore_jerk()
{
    if (_ltrn_psc_jerk == nullptr || !_ltrn_jerk_overridden) {
        return;
    }
    _ltrn_psc_jerk->set(_ltrn_saved_jerk_msss);
    _ltrn_jerk_overridden = false;
}

// push the raised position controller limits for this loop.  They are held at full
// L-turn authority through every active state, including the EXITING stop, which needs
// the whole ceiling, and are restored in one step at release.  That step is benign:
// release happens at or below LTRN_EXIT_V, and AC_Loiter drives PosControl through
// set_pos_vel_accel_xy(), which does not use these kinematic shaping limits.
void ModeLoiter::ltrn_apply_limits()
{
    const float speed_cms = MAX(g2.ltrn_lat_spd, fabsf(_ltrn_entry_vx_ms)) * 100.0f;
    const float accel_cmss = _ltrn_ceiling_ms * 100.0f;

    ltrn_set_jerk(g2.ltrn_jerk);

    // set_max_speed_accel_xy re-derives the jerk limit from PSC_JERK_XY, so it must be
    // called after ltrn_set_jerk to pick up the new value
    pos_control->set_max_speed_accel_xy(MAX(speed_cms, 100.0f), MAX(accel_cmss, LTRN_ACCEL_MIN_MSS * 100.0f));
}

// returns the abort reason, or LTurnExit::NONE if the manoeuvre may continue
ModeLoiter::LTurnExit ModeLoiter::ltrn_abort_reason() const
{
    if (copter.failsafe.radio || !rc().has_ever_seen_rc_input()) {
        return LTurnExit::RC_LOSS;
    }
    if (!motors->armed() || copter.ap.land_complete) {
        return LTurnExit::LANDED;
    }
    if (copter.failsafe.ekf) {
        return LTurnExit::EKF_FAILSAFE;
    }
    if (copter.battery.has_failsafed()) {
        return LTurnExit::BATT_FAILSAFE;
    }
    return LTurnExit::NONE;
}

// all trigger conditions, evaluated only while INACTIVE
bool ModeLoiter::ltrn_trigger_check(float roll_in, float pitch_in) const
{
    if (g2.ltrn_enable != 1) {
        return false;
    }
    if (ltrn_abort_reason() != LTurnExit::NONE) {
        return false;
    }

    const float roll_mag = fabsf(roll_in);
    if (roll_mag < g2.ltrn_roll_trg) {
        return false;
    }
    // roll dominance: lets the pilot sweep from pitch into roll without snapping through centre
    if (roll_mag < fabsf(pitch_in) + g2.ltrn_roll_dom) {
        return false;
    }

    // recent forward command: an L-Turn is a corner out of commanded forward flight.  In
    // log_0084 turns fired on roll alone with the pitch stick centred for over 1.5 s, at
    // which point the remaining speed was drift, not a run-in.
    if (is_positive(g2.ltrn_pit_arm) &&
        (_ltrn_last_pitch_ms == 0 || millis() - _ltrn_last_pitch_ms > LTRN_PIT_ARM_MS)) {
        return false;
    }

    // body-frame speeds use the current heading; nothing is latched yet
    const Vector2f vel_ne_ms = inertial_nav.get_velocity_xy_cms() * 0.01f;
    const float cos_yaw = ahrs.cos_yaw();
    const float sin_yaw = ahrs.sin_yaw();
    const float vx_ms = vel_ne_ms.x * cos_yaw + vel_ne_ms.y * sin_yaw;
    const float vy_ms = -vel_ne_ms.x * sin_yaw + vel_ne_ms.y * cos_yaw;
    if (fabsf(vx_ms) < g2.ltrn_min_spd) {
        return false;
    }
    // forward dominance: the speed being turned must actually be forward speed.  A
    // vehicle sliding sideways as fast as it is moving forwards is already mid-manoeuvre;
    // turning there latches a heading whose body-X is not the direction of travel.
    return fabsf(vx_ms) >= g2.ltrn_fwd_dom * fabsf(vy_ms);
}

// latch the heading and entry speed and take over the horizontal axes.
// fresh_entry is false when re-arming out of an interrupted EXITING blend: the
// position controller limits saved on the original entry must be kept, and the
// lateral target carries the velocity the vehicle already has.
void ModeLoiter::ltrn_enter(float roll_in, bool fresh_entry)
{
    _ltrn_yaw_cd = ahrs.yaw_sensor;
    _ltrn_cos_yaw = ahrs.cos_yaw();
    _ltrn_sin_yaw = ahrs.sin_yaw();

    const Vector2f vel_body_ms = ltrn_body_velocity_ms();
    _ltrn_entry_vx_ms = vel_body_ms.x;

    if (fresh_entry) {
        // body-Y is under active control from t=0; start the ramp from rest so the
        // slide does not inherit whatever lateral drift happened to be present
        _ltrn_vy_target_ms = 0.0f;
        // both feed forwards ramp in from zero
        _ltrn_ax_ff_ms = 0.0f;
        _ltrn_ay_ff_ms = 0.0f;

        // save the position controller limits we are about to raise
        _ltrn_saved_speed_cms = pos_control->get_max_speed_xy_cms();
        _ltrn_saved_accel_cmss = pos_control->get_max_accel_xy_cmss();
        if (_ltrn_psc_jerk != nullptr && !_ltrn_jerk_overridden) {
            _ltrn_saved_jerk_msss = _ltrn_psc_jerk->get();
        }
    } else {
        // carry the current lateral velocity so the target does not step
        _ltrn_vy_target_ms = vel_body_ms.y;
    }

    _ltrn_brake_latched_off = false;
    _ltrn_handover_start_ms = 0;
    _ltrn_blending = false;
    _ltrn_exit_escalated = false;
    _ltrn_exit_escalate_ms = 0;
    _ltrn_retrig_start_ms = 0;

    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    _ltrn_state = LTurnState::BRAKING;
    _ltrn_state_ms = millis();
    gcs().send_text(MAV_SEVERITY_INFO, "LTRN: %s %s from %.1f m/s",
                    fresh_entry ? "braking" : "re-trigger",
                    is_negative(roll_in) ? "left" : "right", (double)fabsf(_ltrn_entry_vx_ms));
}

// begin the handback.  A normal stick-centred exit becomes an active stop under the
// L-turn's own control (EXITING) and releases once speed is below LTRN_EXIT_V; every
// abort, and LTRN_EXIT_T of zero, releases immediately.
void ModeLoiter::ltrn_begin_exit(LTurnExit reason, float roll_in)
{
    const bool is_abort = (reason != LTurnExit::STICK_CENTRED);
    if (!is_abort && is_positive(g2.ltrn_exit_t)) {
        // The stop writes its trajectory straight into PosControl's desired state (see
        // ltrn_update), so it must start from exactly that state or it moves the step
        // from the release to here.  Seed velocity from PosControl's _vel_desired and
        // the feed forward from its _accel_desired - not from the measured velocity and
        // LTRN's own feed forward, which in log_0076 were up to 1.4 m/s and 3.5 m/s/s
        // away from what PosControl was actually holding.
        _ltrn_exit_vel_ms = ltrn_ne_to_body(pos_control->get_vel_desired_cms().xy() * 0.01f);
        const Vector2f accel_des_body_ms = ltrn_ne_to_body(pos_control->get_accel_desired_cmss().xy() * 0.01f);
        _ltrn_ax_ff_ms = accel_des_body_ms.x;
        _ltrn_ay_ff_ms = accel_des_body_ms.y;
        _ltrn_exit_dist_m = 0.0f;
        _ltrn_handover_start_ms = 0;
        _ltrn_blending = false;
        _ltrn_exit_escalated = false;
        _ltrn_exit_escalate_ms = 0;
        _ltrn_retrig_start_ms = 0;
        // evaluated with the same test as ltrn_update() on this same loop
        _ltrn_exit_x_done = false;

        _ltrn_state = LTurnState::EXITING;
        _ltrn_state_ms = millis();
        _ltrn_exit_start_ms = _ltrn_state_ms;
        gcs().send_text(MAV_SEVERITY_INFO, "LTRN: stopping from %.1f m/s a %.1f j %.1f",
                        (double)ltrn_body_velocity_ms().length(),
                        (double)MIN(MAX(g2.ltrn_exit_acc, LTRN_ACCEL_MIN_MSS), _ltrn_ceiling_ms),
                        (double)MAX(MIN(g2.ltrn_exit_jerk, pos_control->get_max_jerk_xy_cmsss() * 0.01f), LTRN_JERK_MIN_MSSS));
        return;
    }
    ltrn_finish_exit(reason, roll_in);
}

// restore the borrowed position controller limits and re-seed loiter_nav from the
// vehicle's current position and velocity so the handback has no positional jump
void ModeLoiter::ltrn_finish_exit(LTurnExit reason, float roll_in)
{
#if HAL_LOGGING_ENABLED
    // Guaranteed final record, written on the very loop the exit fires and carrying the
    // roll value the exit test actually compared.  Without this the last LTRN record in
    // the log is the previous loop's, which is what made exits look impossible.
    // Targets and raw demand are the last loop's; they used to be written as zeros,
    // which read as a commanded step that never happened.
    const Vector2f vel_body_ms = ltrn_body_velocity_ms();
    ltrn_log(roll_in, vel_body_ms, _ltrn_last_vel_target_ms, _ltrn_last_accel_raw_ms,
             Vector2f{_ltrn_ax_ff_ms, _ltrn_ay_ff_ms}, reason);
#endif

    ltrn_restore_jerk();
    pos_control->set_max_speed_accel_xy(_ltrn_saved_speed_cms, _ltrn_saved_accel_cmss);
    // Deliberately NOT loiter_nav->init_target().  That path runs
    // relax_velocity_controller_xy() -> init_xy_controller(), which resets _pos_target
    // to the measured position, _vel_desired to the measured velocity, zeroes
    // _accel_desired and re-seeds the velocity integrator - the same discontinuity as
    // at entry, in reverse.  AC_Loiter::calc_desired_velocity() already continues from
    // PosControl's _pos_desired / _vel_desired, and run() keeps loiter_nav's pilot
    // acceleration model live every loop, so the handover is continuous without it.
    // Non-flying exits are re-seeded by run()'s own landed/motor-stopped branches.

    _ltrn_state = LTurnState::INACTIVE;
    _ltrn_state_ms = millis();
    _ltrn_vy_target_ms = 0.0f;
    _ltrn_ax_ff_ms = 0.0f;
    _ltrn_ay_ff_ms = 0.0f;
    _ltrn_brake_latched_off = false;
    _ltrn_exit_vel_ms.zero();
    _ltrn_exit_dist_m = 0.0f;
    _ltrn_exit_x_done = false;
    _ltrn_handover_start_ms = 0;
    _ltrn_last_vel_target_ms.zero();
    _ltrn_last_accel_raw_ms.zero();
    _ltrn_blending = false;

    // a timeout or distance release is not an abort, but it means LTRN_EXIT_T was too short to finish the
    // stop, so it is raised as a warning alongside the aborts
    const bool normal = (reason == LTurnExit::STICK_CENTRED || reason == LTurnExit::STOPPED ||
                         reason == LTurnExit::STICK_BLEND);
    gcs().send_text(normal ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
                    "LTRN: exit (%s) spd %.2f RIn %.2f", ltrn_exit_name(reason),
                    (double)ltrn_body_velocity_ms().length(), (double)roll_in);
}

// run the L-turn state machine.  Returns true if it has taken over the horizontal
// axes this loop, in which case the caller must not run loiter_nav.
bool ModeLoiter::ltrn_update()
{
    // one-shot parameter lookups; AP_Param::find() is a search, so it is never run per loop
    if (!_ltrn_params_found) {
        _ltrn_params_found = true;
        enum ap_var_type ptype;
        AP_Param *vp = AP_Param::find("PSC_JERK_XY", &ptype);
        if (vp != nullptr && ptype == AP_PARAM_FLOAT) {
            _ltrn_psc_jerk = (AP_Float *)vp;
        }
        vp = AP_Param::find("LOIT_ANG_MAX", &ptype);
        if (vp != nullptr && ptype == AP_PARAM_FLOAT) {
            _ltrn_loit_ang_max = (AP_Float *)vp;
        }
    }

    const float roll_in = ltrn_roll_input();
    const float pitch_in = ltrn_pitch_input();

    // arming history for the trigger's recent-forward-command test.  Updated in every
    // state, including INACTIVE, so the window is always the true last use of the stick.
    if (fabsf(pitch_in) > g2.ltrn_pit_arm) {
        _ltrn_last_pitch_ms = MAX(millis(), 1U);
    }

    if (_ltrn_state == LTurnState::INACTIVE) {
        if (!ltrn_trigger_check(roll_in, pitch_in)) {
            return false;
        }
        ltrn_enter(roll_in, true);
    } else {
        // aborts always take precedence and always hand back immediately
        const LTurnExit abort_reason = ltrn_abort_reason();
        if (abort_reason != LTurnExit::NONE) {
            ltrn_finish_exit(abort_reason, roll_in);
            return false;
        }
        // A fresh valid trigger during the exit stop re-arms the manoeuvre at once,
        // carrying the current velocity as the new entry state.  On top of the normal
        // trigger the roll input must oppose the current lateral velocity for
        // LTRN_RETRIG_MS: a pilot countering the exit's lateral overrun holds exactly
        // that stick, and in log_0084 those corrections re-armed the turn they were
        // trying to stop.  A deliberate reversal holds the stick long enough to pass.
        if (_ltrn_state == LTurnState::EXITING) {
            const float vy_now_ms = ltrn_body_velocity_ms().y;
            const bool reversing = is_negative(roll_in * vy_now_ms);
            if (reversing) {
                if (_ltrn_retrig_start_ms == 0) {
                    _ltrn_retrig_start_ms = MAX(millis(), 1U);
                }
            } else {
                _ltrn_retrig_start_ms = 0;
            }
            if (_ltrn_retrig_start_ms != 0 &&
                millis() - _ltrn_retrig_start_ms >= LTRN_RETRIG_MS &&
                ltrn_trigger_check(roll_in, pitch_in)) {
                ltrn_enter(roll_in, false);
            }
        }
        // pilot has re-centred the roll stick: begin the handback
        if (_ltrn_state != LTurnState::EXITING && fabsf(roll_in) < g2.ltrn_dz) {
            ltrn_begin_exit(LTurnExit::STICK_CENTRED, roll_in);
            if (_ltrn_state == LTurnState::INACTIVE) {
                return false;
            }
        }
    }

    ltrn_update_accel_limits();
    // raised limits, held for the whole manoeuvre including the exit stop.  Applied before
    // the jerk limit below is read: set_max_speed_accel_xy() is what derives PosControl's
    // effective jerk from them.
    ltrn_apply_limits();

    const Vector2f vel_body_ms = ltrn_body_velocity_ms();
    const float dt = MAX(G_Dt, 0.001f);

    // Settled once the state machine above has run, so a stop that began on this very
    // loop already gets the exit limits rather than the entry brake's.
    const bool exiting = (_ltrn_state == LTurnState::EXITING);

    // One jerk limit for everything the feed forward does.  The entry brake and the
    // lateral phase slew at LTRN_JERK; the exit stop, including the stick blend, slews at
    // the gentler LTRN_EXIT_JERK - in log_0079/0081 the Y stage unwound its 19 deg of
    // opposing lean at the entry jerk and peaked the roll rate demand at ATC_RATE_R_MAX.
    // Either parameter is capped at the jerk PosControl itself allows after
    // set_max_speed_accel_xy(): ATC_RATE_R/P_MAX x g, and 0.5 x sqrt(accel x
    // ATC_ACCEL_R/P_MAX x g).  The feed forward used to slew at the raw parameter, which in
    // log_0078 (30 m/s/s/s) asked for ~175 deg/s of lean rate against a 90 deg/s limit;
    // the attitude loop fell 100-170 ms behind and roll overshot.
    // The gentler exit jerk applies only to the brake.  A feed forward carried over from
    // the lateral phase still points along the velocity it created: that is the pilot's
    // own acceleration, and removing it slowly is what let the vehicle keep accelerating
    // sideways for 0.7 s after the stick was centred in log_0084.  While either axis'
    // feed forward is still driving the vehicle along that axis it is removed at
    // LTRN_JERK; the exit jerk takes over once the feed forward is zero or opposing.
    const bool ff_driving = exiting && (is_positive(_ltrn_ax_ff_ms * vel_body_ms.x) ||
                                        is_positive(_ltrn_ay_ff_ms * vel_body_ms.y));
    // an escalated stop has run out of distance or time and stops at full authority
    const float exit_jerk_param = g2.ltrn_exit_jerk * (_ltrn_exit_escalated ? 2.0f : 1.0f);
    const float jerk_param = (exiting && !ff_driving) ? exit_jerk_param : g2.ltrn_jerk;
    const float jerk_limit_msss = MAX(MIN(jerk_param, pos_control->get_max_jerk_xy_cmsss() * 0.01f), LTRN_JERK_MIN_MSSS);
    if (!exiting && !_ltrn_jerk_cap_warned && jerk_limit_msss < g2.ltrn_jerk - 0.1f) {
        _ltrn_jerk_cap_warned = true;
        gcs().send_text(MAV_SEVERITY_WARNING, "LTRN: LTRN_JERK capped to %.1f m/s/s/s", (double)jerk_limit_msss);
    }

    // The exit stop leans no harder than LTRN_EXIT_ACC.  It can only lower the lean: the
    // L-Turn ceiling derived from the lean angle limits remains the hard cap.  The entry
    // brake, the lateral phase and the handover reference brake are unaffected.
    const float exit_ceiling_ms = _ltrn_exit_escalated
                                  ? _ltrn_ceiling_ms
                                  : MIN(MAX(g2.ltrn_exit_acc, LTRN_ACCEL_MIN_MSS), _ltrn_ceiling_ms);
    const float max_delta_ms = jerk_limit_msss * dt;
    const float exit_v_ms = MAX(g2.ltrn_exit_v, 0.05f);

    // pilot stick acceleration exactly as AC_Loiter will apply it on release; run() refreshes
    // it with set_pilot_desired_acceleration() every loop before calling this
    const Vector2f pilot_accel_ne_ms = loiter_nav->get_pilot_desired_acceleration() * 0.01f;
    const bool stick_active = pilot_accel_ne_ms.length() > LTRN_BLEND_STICK_MIN_MSS;

    // EXITING release.  Once measured horizontal speed is below LTRN_EXIT_V the stop
    // releases as soon as PosControl's desired state matches what Loiter will continue
    // with (see LTRN_HANDOVER_*), or after LTRN_HANDOVER_TIMEOUT_MS regardless.  Before
    // that, the stop is abandoned once it has travelled LTRN_EXIT_D or LTRN_EXIT_T has
    // run out; a stop that has not finished by then is no longer helping the shape.
    if (_ltrn_state == LTurnState::EXITING) {
        const float speed_ms = vel_body_ms.length();
        _ltrn_exit_dist_m += speed_ms * dt;
        const uint32_t now_ms = millis();
        if (_ltrn_handover_start_ms == 0 && speed_ms <= exit_v_ms) {
            _ltrn_handover_start_ms = MAX(now_ms, 1U);
        }
        if (_ltrn_handover_start_ms != 0) {
            // Loiter keeps _vel_desired and continues from it, and overwrites _accel_desired
            // with its pilot demand minus its brake.  With the sticks centred the reference
            // is the brake law the Y stage already lands on: PSC_VELXY_P / 2 x speed,
            // opposing the desired velocity.  With a stick held it is the pilot demand
            // itself, which the exit blend below slews the feed forward onto - in log_0079
            // every exit had pitch held and Loiter stepped the demand by 4.5-4.95 m/s/s.
            const Vector2f vel_des_ms = pos_control->get_vel_desired_cms().xy() * 0.01f;
            const Vector2f accel_des_ms = pos_control->get_accel_desired_cmss().xy() * 0.01f;
            const Vector2f vel_meas_ms = inertial_nav.get_velocity_xy_cms() * 0.01f;
            Vector2f accel_ref_ms;
            if (stick_active) {
                accel_ref_ms = pilot_accel_ne_ms;
            } else {
                const float vel_des_len = vel_des_ms.length();
                if (is_positive(vel_des_len)) {
                    const float brake_ms = MIN(pos_control->get_vel_xy_pid().kP() * 0.5f * vel_des_len, _ltrn_ceiling_ms);
                    accel_ref_ms = vel_des_ms * (-brake_ms / vel_des_len);
                }
            }
            // while accelerating the aircraft trails its desired velocity by about
            // accel x lag; Loiter inherits that tracking error unchanged, it is not a step
            const float vel_tol_ms = LTRN_HANDOVER_VEL_TOL_MS + accel_ref_ms.length() * LTRN_HANDOVER_LAG_S;
            const float accel_gap_ms = (accel_des_ms - accel_ref_ms).length();
            if ((vel_des_ms - vel_meas_ms).length() <= vel_tol_ms &&
                accel_gap_ms <= LTRN_HANDOVER_ACC_TOL_MSS) {
                ltrn_finish_exit(stick_active ? LTurnExit::STICK_BLEND : LTurnExit::STOPPED, roll_in);
                return false;
            }
            // a blend also gets the time its remaining slew needs, capped so a stick that
            // keeps moving cannot hold the state machine in the exit
            const float blend_allow_ms = stick_active ? MIN(1000.0f * accel_gap_ms / jerk_limit_msss, LTRN_BLEND_EXTRA_MAX_MS) : 0.0f;
            if (now_ms - _ltrn_handover_start_ms >= LTRN_HANDOVER_TIMEOUT_MS + (uint32_t)blend_allow_ms) {
                ltrn_finish_exit(LTurnExit::HANDOVER_TIMEOUT, roll_in);
                return false;
            }
        } else if (!_ltrn_exit_escalated &&
                   (_ltrn_exit_dist_m > g2.ltrn_exit_d ||
                    (now_ms - _ltrn_exit_start_ms) * 0.001f >= g2.ltrn_exit_t)) {
            // Out of distance or time, but still too fast to hand over.  Releasing here is
            // what stepped PosControl's desired acceleration from the 2 m/s/s brake to the
            // pilot's 5 m/s/s pitch demand in one loop in log_0084, every single time.
            // Instead the stop keeps control and finishes at full authority; the release
            // is then always the normal handover, through the slewed blend.
            _ltrn_exit_escalated = true;
            _ltrn_exit_escalate_ms = MAX(now_ms, 1U);
            gcs().send_text(MAV_SEVERITY_INFO, "LTRN: stop escalated");
            if (stick_active) {
                // with a stick held there is nothing to be gained by stopping first: open
                // the handover window now so the feed forward slews onto the pilot's
                // demand from here.  The vector clamp keeps the blend inside the ceiling
                // even though the vehicle is still moving laterally.
                _ltrn_handover_start_ms = MAX(now_ms, 1U);
            }
        } else if (_ltrn_exit_escalated &&
                   (now_ms - _ltrn_exit_escalate_ms) * 0.001f >= 2.0f * g2.ltrn_exit_t) {
            // hard backstop: the state machine can never hang in an escalated stop
            ltrn_finish_exit(LTurnExit::EXIT_TIMEOUT, roll_in);
            return false;
        }
    }

    // Exit blend: inside the handover window with a stick held, the feed forward slews onto
    // the pilot's demand instead of continuing the stop, so Loiter's overwrite at release
    // changes nothing.  Before the window the stop runs unchanged whatever the sticks do.
    const bool was_blending = _ltrn_blending;
    _ltrn_blending = (_ltrn_state == LTurnState::EXITING && _ltrn_handover_start_ms != 0 && stick_active);
    if (_ltrn_blending && !was_blending) {
        gcs().send_text(MAV_SEVERITY_INFO, "LTRN: blending into stick %.1f m/s/s", (double)pilot_accel_ne_ms.length());
    }
    const Vector2f pilot_accel_body_ms = ltrn_ne_to_body(pilot_accel_ne_ms);
    // the pilot may command up to LOIT_ANG_MAX, above both the exit ceiling and the
    // LTRN_MARGIN ceiling; the blend must be able to reach it or the release would still
    // step by the difference.  Outside the blend the exit stop is held to LTRN_EXIT_ACC.
    const float accel_limit_ms = _ltrn_blending ? MAX(exit_ceiling_ms, pilot_accel_body_ms.length())
                                                 : (exiting ? exit_ceiling_ms : _ltrn_ceiling_ms);

    // BRAKING -> LATERAL once the body-X speed has fallen to LTRN_XOVER of the entry
    // speed.  The floor keeps LTRN_XOVER = 0 (stop, dwell, then go) from hanging.
    if (_ltrn_state == LTurnState::BRAKING) {
        const float xover_ms = MAX(g2.ltrn_xover * fabsf(_ltrn_entry_vx_ms), LTRN_XOVER_MIN_MS);
        if (fabsf(vel_body_ms.x) <= xover_ms) {
            _ltrn_state = LTurnState::LATERAL;
            _ltrn_state_ms = millis();
            gcs().send_text(MAV_SEVERITY_INFO, "LTRN: lateral");
        }
    }

    // ---- form the raw body-frame acceleration demand ----
    Vector2f accel_raw_ms;
    Vector2f vel_target_body_ms;

    if (_ltrn_state == LTurnState::EXITING) {
        // Axis-prioritised active stop.  Opposing the resultant velocity (round 6) kept
        // both components alive together and flew a curve; here body-X is stopped first
        // and only then does body-Y get any stopping authority, so the tail is a straight
        // line along one body axis.
        //
        // The command is a kinematically consistent stop trajectory: the velocity target
        // is _ltrn_exit_vel_ms and the feed forward is its deceleration, and the
        // trajectory is advanced by the feed forward actually commanded (below).  A flat
        // zero velocity target plus a feed forward would make PosControl's shaper add a
        // hidden correction of its own towards zero, which loiter would overwrite on the
        // first handover loop.
        //
        // Let the stop follow the aircraft rather than the other way round.  The aircraft
        // leans 3-7 deg past the commanded brake (log_0081), so it stops short of the planned
        // trajectory; PosControl then pulled it forward - 1.7-2.2 m/s/s against the brake,
        // roughly half position error and half velocity gap - and the lean reversed by a
        // median 11-16 deg right at the end of the turn.  Per body axis, whenever the
        // aircraft is slower than the trajectory in the same direction, or already through
        // zero, the trajectory velocity eases down to it with LTRN_EXIT_TC.  It never adds
        // speed, and the stop laws below then taper the brake from the eased value.  The
        // stick blend is exempt: there the aircraft trails its target by design.
        if (is_positive(g2.ltrn_exit_tc) && !_ltrn_blending) {
            const float k = MIN(dt / g2.ltrn_exit_tc, 1.0f);
            const float meas_ms[2] = { vel_body_ms.x, vel_body_ms.y };
            float traj_ms[2] = { _ltrn_exit_vel_ms.x, _ltrn_exit_vel_ms.y };
            for (uint8_t i = 0; i < 2; i++) {
                const float follow_ms = (meas_ms[i] * traj_ms[i] > 0.0f) ? meas_ms[i] : 0.0f;
                if (fabsf(follow_ms) < fabsf(traj_ms[i])) {
                    traj_ms[i] += (follow_ms - traj_ms[i]) * k;
                }
            }
            _ltrn_exit_vel_ms = Vector2f{ traj_ms[0], traj_ms[1] };
        }

        // X stage ends when measured forward speed is inside LTRN_VXDB, or when the X
        // trajectory has itself reached rest - its demand is then zero, and the lateral
        // axis takes the whole acceleration circle rather than its LTRN_EXIT_YSHR share.
        // The X trajectory is no longer clamped to the measured forward speed here: it is
        // PosControl's _vel_desired, so a clamp would be a velocity target step.  The X law below keeps driving it to
        // rest through the jerk slew instead.
        if (!_ltrn_exit_x_done &&
            (fabsf(vel_body_ms.x) <= g2.ltrn_vxdb || is_zero(_ltrn_exit_vel_ms.x))) {
            _ltrn_exit_x_done = true;
            _ltrn_state_ms = millis();
        }

        // body-X law, both stages: the fastest stop to zero the LTRN_EXIT_JERK slew can
        // follow - full exit ceiling while there is speed to spare, tapering at exactly J
        // so it reaches zero speed and zero acceleration together.
        const float vx_t_ms = _ltrn_exit_vel_ms.x;
        const float ax_mag_ms = MIN(exit_ceiling_ms, sqrtf(2.0f * jerk_limit_msss * fabsf(vx_t_ms)));
        accel_raw_ms.x = is_positive(vx_t_ms) ? -ax_mag_ms : (is_negative(vx_t_ms) ? ax_mag_ms : 0.0f);

        // body-Y law, both stages.  During the X stage it is scaled by LTRN_EXIT_YSHR and
        // the vector clamp gives it only what body-X leaves of the acceleration circle;
        // the tail is no longer a perfectly straight line along one body axis, which buys
        // back the 0.6-1.3 s the lateral axis used to spend coasting (log_0084).
        // In trajectory speed v:
        //   v >  LTRN_EXIT_V: sqrt(a_rel^2 + 2 J (v - LTRN_EXIT_V)), capped at the exit ceiling
        //   v <= LTRN_EXIT_V: a_rel * v / LTRN_EXIT_V
        // a_rel is loiter's own brake law at the release speed (AC_Loiter uses
        // brake_gain = PSC_VELXY_P / 2), so the stop arrives at LTRN_EXIT_V decelerating
        // at the rate loiter's brake settles to rather than at the ceiling.
        {
            const float vy_t_ms = _ltrn_exit_vel_ms.y;
            const float vy_abs_ms = fabsf(vy_t_ms);
            const float a_rel_ms = MIN(pos_control->get_vel_xy_pid().kP() * 0.5f * exit_v_ms, exit_ceiling_ms);
            float ay_mag_ms;
            if (vy_abs_ms > exit_v_ms) {
                ay_mag_ms = sqrtf(a_rel_ms * a_rel_ms + 2.0f * jerk_limit_msss * (vy_abs_ms - exit_v_ms));
            } else {
                ay_mag_ms = a_rel_ms * vy_abs_ms / exit_v_ms;
            }
            ay_mag_ms = MIN(ay_mag_ms, exit_ceiling_ms);
            // during the X stage the lateral brake runs at LTRN_EXIT_YSHR of the full law
            // rather than at zero; body-X is still served first by the vector clamp below,
            // so this only uses the part of the acceleration circle body-X is not using.
            if (!_ltrn_exit_x_done) {
                ay_mag_ms *= constrain_float(g2.ltrn_exit_yshr, 0.0f, 1.0f);
            }
            accel_raw_ms.y = is_positive(vy_t_ms) ? -ay_mag_ms : (is_negative(vy_t_ms) ? ay_mag_ms : 0.0f);
        }
        // the exit blend overrides both stop laws: the demand is the pilot's acceleration,
        // reached through the same jerk slew as everything else
        if (_ltrn_blending) {
            accel_raw_ms = pilot_accel_body_ms;
        }
        // vel_target_body_ms is taken after the trajectory is advanced, below
    } else {
        // body-X: zero velocity target throughout.  The feed forward is what makes this
        // a brake rather than a decay.  Once the forward speed falls inside LTRN_VXDB the
        // brake latches off for good, which stops the feed forward chattering at the zero
        // crossing; the release is ramped by the slew below, not stepped.
        if (fabsf(vel_body_ms.x) < g2.ltrn_vxdb) {
            _ltrn_brake_latched_off = true;
        }
        if (!_ltrn_brake_latched_off) {
            accel_raw_ms.x = is_negative(_ltrn_entry_vx_ms) ? _ltrn_decel_ms : -_ltrn_decel_ms;
        }

        // body-Y: the target follows the roll stick from the moment of trigger in both
        // BRAKING and LATERAL.  The target stays under active control throughout - only
        // the acceleration budget differs between phases, and that is decided by the
        // speed-scheduled split below.
        float vy_cmd_ms = constrain_float(roll_in, -1.0f, 1.0f) * g2.ltrn_lat_spd;
        // The target used to ramp at LTRN_LAT_ACC whether the vehicle was following or
        // not.  The lead it built up is speed the vehicle is still gaining when the stick
        // is centred, and in log_0084 it flew off as ~2 m/s of overshoot past
        // LTRN_LAT_SPD.  Hold the target within LTRN_LAT_LEAD of the measured speed while
        // it is growing; a target being reduced towards the stick is never clamped, so
        // releasing the stick is as immediate as before.
        if (fabsf(vy_cmd_ms) > fabsf(_ltrn_vy_target_ms)) {
            const float lead_ms = MAX(g2.ltrn_lat_lead, 0.1f);
            vy_cmd_ms = constrain_float(vy_cmd_ms, vel_body_ms.y - lead_ms, vel_body_ms.y + lead_ms);
        }
        const float dv_max_ms = _ltrn_lat_acc_ms * dt;
        const float dv_ms = constrain_float(vy_cmd_ms - _ltrn_vy_target_ms, -dv_max_ms, dv_max_ms);
        _ltrn_vy_target_ms += dv_ms;
        accel_raw_ms.y = dv_ms / dt;
        vel_target_body_ms.y = _ltrn_vy_target_ms;
    }

    // ---- clamp the demand as a VECTOR, splitting the budget by LTRN_XPRIO ----
    Vector2f accel_cmd_ms = accel_raw_ms;
    const float demand_mag = accel_raw_ms.length();
    // accel_limit_ms is the L-Turn ceiling, LTRN_EXIT_ACC during the exit stop, and the
    // pilot's own demand during the exit blend
    const bool saturated = (demand_mag > accel_limit_ms);
    if (saturated && _ltrn_state == LTurnState::EXITING) {
        // same circle, body-X served first: X gets its whole demand up to the ceiling
        // and Y gets whatever is left inside the circle.  In the X stage the Y demand is
        // LTRN_EXIT_YSHR of the lateral law, so what it actually receives there is only
        // the part of the circle the body-X brake is not using.
        accel_cmd_ms.x = constrain_float(accel_raw_ms.x, -accel_limit_ms, accel_limit_ms);
        const float ay_sq = accel_limit_ms * accel_limit_ms - accel_cmd_ms.x * accel_cmd_ms.x;
        const float ay_budget = is_positive(ay_sq) ? sqrtf(ay_sq) : 0.0f;
        accel_cmd_ms.y = constrain_float(accel_raw_ms.y, -ay_budget, ay_budget);
    } else if (saturated) {
        // The brake's share of the ceiling decays with the forward speed still to be
        // killed, so the budget transfers smoothly to the lateral axis instead of
        // stepping at the crossover.  LTRN_XPRIO is the MAXIMUM brake share, taken at
        // and above the handover speed; LTRN_XOVER sets that handover speed as a
        // fraction of the entry speed.
        const float handover_ms = MAX(g2.ltrn_min_spd, fabsf(_ltrn_entry_vx_ms) * g2.ltrn_xover);
        const float speed_frac = constrain_float(fabsf(vel_body_ms.x) / MAX(handover_ms, 0.1f), 0.0f, 1.0f);
        float brake_share = constrain_float(g2.ltrn_xprio, 0.0f, 1.0f) * speed_frac;
        // Floor: while there is forward speed left to kill, the brake never gets less
        // than LTRN_XFLOOR of the ceiling.  Without it the share fell with VX and the
        // last few m/s were never killed, leaving them for the exit.
        if (!_ltrn_brake_latched_off && fabsf(vel_body_ms.x) > g2.ltrn_vxdb) {
            brake_share = MAX(brake_share, constrain_float(g2.ltrn_xfloor, 0.0f, 1.0f));
        }
        // the brake axis is served first, up to its scheduled share of the ceiling
        const float ax_budget = _ltrn_ceiling_ms * brake_share;
        accel_cmd_ms.x = constrain_float(accel_raw_ms.x, -ax_budget, ax_budget);
        // whatever magnitude is left in the circle goes to the lateral axis
        const float ay_sq = _ltrn_ceiling_ms * _ltrn_ceiling_ms - accel_cmd_ms.x * accel_cmd_ms.x;
        const float ay_budget = is_positive(ay_sq) ? sqrtf(ay_sq) : 0.0f;
        accel_cmd_ms.y = constrain_float(accel_raw_ms.y, -ay_budget, ay_budget);
    }

    const Vector2f &slew_target_ms = accel_cmd_ms;

    // ---- slew the feed forward as a VECTOR ----
    // Limiting each axis separately let both slew at the full rate at once, giving
    // sqrt(2) x the jerk limit on the diagonal.  The change in the acceleration vector is
    // now scaled by a single factor so its magnitude never exceeds jerk_limit_msss x dt -
    // LTRN_JERK during the entry brake and lateral phases, LTRN_EXIT_JERK during the exit.
    const float ff_prev_mag = Vector2f{_ltrn_ax_ff_ms, _ltrn_ay_ff_ms}.length();
    Vector2f ff_step_ms{slew_target_ms.x - _ltrn_ax_ff_ms, slew_target_ms.y - _ltrn_ay_ff_ms};
    const float ff_step_mag = ff_step_ms.length();
    if (ff_step_mag > max_delta_ms && is_positive(ff_step_mag)) {
        ff_step_ms *= max_delta_ms / ff_step_mag;
    }
    _ltrn_ax_ff_ms += ff_step_ms.x;
    _ltrn_ay_ff_ms += ff_step_ms.y;

    // safety net: re-clamp the magnitude so the commanded vector never exceeds the
    // ceiling.  Slewing from inside the circle towards a target inside it cannot leave
    // it, but the ceiling can shrink between loops (load, LTRN_MARGIN).  A feed forward
    // that is already above the ceiling - EXITING seeds it from PosControl's
    // _accel_desired, which can be - is only prevented from growing, and comes back
    // inside through the slew rather than being stepped onto the ceiling.
    Vector2f accel_ff_body_ms{_ltrn_ax_ff_ms, _ltrn_ay_ff_ms};
    const float ff_mag = accel_ff_body_ms.length();
    const float ff_limit = MAX(accel_limit_ms, ff_prev_mag);
    if (ff_mag > ff_limit && is_positive(ff_mag)) {
        accel_ff_body_ms *= ff_limit / ff_mag;
        _ltrn_ax_ff_ms = accel_ff_body_ms.x;
        _ltrn_ay_ff_ms = accel_ff_body_ms.y;
    }

    // advance the stop trajectory by the feed forward actually commanded, so velocity
    // target and feed forward stay kinematically consistent.  Each axis is handled on
    // its own and never reverses through zero: a step that would cross zero, or leave
    // an axis already at rest, ends that axis at rest.  The exit blend is exempt: the
    // pilot's demand may legitimately take an axis through zero and away from rest.
    if (_ltrn_state == LTurnState::EXITING) {
        Vector2f next_ms = _ltrn_exit_vel_ms + accel_ff_body_ms * dt;
        if (!_ltrn_blending) {
            if (next_ms.x * _ltrn_exit_vel_ms.x <= 0.0f) {
                next_ms.x = 0.0f;
            }
            if (next_ms.y * _ltrn_exit_vel_ms.y <= 0.0f) {
                next_ms.y = 0.0f;
            }
        }
        _ltrn_exit_vel_ms = next_ms;
        vel_target_body_ms = _ltrn_exit_vel_ms;
    }

    // rotate the feed forward into NE
    const Vector2f accel_ff_ne_cmss{
        (accel_ff_body_ms.x * _ltrn_cos_yaw - accel_ff_body_ms.y * _ltrn_sin_yaw) * 100.0f,
        (accel_ff_body_ms.x * _ltrn_sin_yaw + accel_ff_body_ms.y * _ltrn_cos_yaw) * 100.0f };

    // rotate the velocity demand into NE
    Vector2f vel_target_ne_cms{
        (vel_target_body_ms.x * _ltrn_cos_yaw - vel_target_body_ms.y * _ltrn_sin_yaw) * 100.0f,
        (vel_target_body_ms.x * _ltrn_sin_yaw + vel_target_body_ms.y * _ltrn_cos_yaw) * 100.0f };

#if AP_AVOIDANCE_ENABLED
    // fence and proximity limiting, matching AC_Loiter::calc_desired_velocity()
    {
        Vector3f avoid_vel_cms{vel_target_ne_cms.x, vel_target_ne_cms.y, 0.0f};
        copter.avoid.adjust_velocity(avoid_vel_cms,
                                     pos_control->get_pos_xy_p().kP(),
                                     pos_control->get_max_accel_xy_cmss(),
                                     pos_control->get_pos_z_p().kP(),
                                     pos_control->get_max_accel_z_cmss(),
                                     dt);
        vel_target_ne_cms.x = avoid_vel_cms.x;
        vel_target_ne_cms.y = avoid_vel_cms.y;
    }
#endif

    // The position target is carried forward from wherever loiter left it:
    // input_vel_accel_xy() integrates it along the manoeuvre's kinematic path and the
    // position loop stays engaged throughout.  This used to call
    // stop_pos_xy_stabilisation() every loop, which set _pos_target to the measured
    // position and so discarded loiter's accumulated position error in one loop at
    // entry - a step of up to PSC_POSXY_P x PSC_VELXY_P x error in acceleration demand,
    // underneath the smooth feed forward.  The carried error is capped by
    // LOITER_POS_CORRECTION_MAX and decays through the normal position loop.
    if (_ltrn_state == LTurnState::EXITING) {
        // The stop is written straight into PosControl's desired state, the same way
        // AC_Loiter::calc_desired_velocity() does, so the state Loiter inherits at
        // release is exactly the jerk-limited stop trajectory.  input_vel_accel_xy()
        // only chased it: in log_0076 _vel_desired trailed the trajectory by 0.7-1.4 m/s
        // and _accel_desired carried a ~4 m/s/s hidden correction, and Loiter's
        // overwrite of both at release was the exit jerk.
        // Any avoidance limit is written back so the trajectory stays PosControl's state.
        _ltrn_exit_vel_ms = ltrn_ne_to_body(vel_target_ne_cms * 0.01f);
        vel_target_body_ms = _ltrn_exit_vel_ms;
        Vector2p pos_desired_cm = pos_control->get_pos_desired_cm().xy() + (vel_target_ne_cms * dt).topostype();
        // Bleed the position error with LTRN_EXIT_TC so a stop that ends short is accepted
        // where it ends, instead of Loiter inheriting 0.55-0.75 m of error (log_0081) and
        // dragging the aircraft forward after release.  The target moves continuously, at
        // most error / LTRN_EXIT_TC per second, so this is never a step.
        if (is_positive(g2.ltrn_exit_tc)) {
            const Vector2f pos_err_cm = (pos_control->get_pos_target_cm().xy() - inertial_nav.get_position_xy_cm().topostype()).tofloat();
            pos_desired_cm -= (pos_err_cm * MIN(dt / g2.ltrn_exit_tc, 1.0f)).topostype();
        }
        pos_control->set_pos_vel_accel_xy(pos_desired_cm, vel_target_ne_cms, accel_ff_ne_cmss);
    } else {
        pos_control->input_vel_accel_xy(vel_target_ne_cms, accel_ff_ne_cmss);
    }
    pos_control->update_xy_controller();

    // hold the latched heading; pilot yaw input is ignored while the manoeuvre runs
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), _ltrn_yaw_cd);

#if HAL_LOGGING_ENABLED
    ltrn_log(roll_in, vel_body_ms, vel_target_body_ms, accel_raw_ms,
             accel_ff_body_ms, LTurnExit::NONE);
    _ltrn_last_vel_target_ms = vel_target_body_ms;
    _ltrn_last_accel_raw_ms = accel_raw_ms;
#endif

    return true;
}

#if HAL_LOGGING_ENABLED
void ModeLoiter::ltrn_log(float roll_in, const Vector2f &vel_body_ms,
                          const Vector2f &vel_target_body_ms, const Vector2f &accel_raw_ms,
                          const Vector2f &accel_out_ms, LTurnExit reason) const
{
    // the exit stop is one state with two stages; the log shows them as 3 and 4
    uint8_t st = (uint8_t)_ltrn_state;
    if (_ltrn_state == LTurnState::EXITING && _ltrn_exit_x_done) {
        st = 4;
    }
    if (_ltrn_state == LTurnState::EXITING && _ltrn_blending) {
        st = 5;
    }
    copter.Log_Write_LTurn(st,
                           (millis() - _ltrn_state_ms) * 0.001f,
                           vel_body_ms.x, vel_body_ms.y,
                           vel_target_body_ms.x, vel_target_body_ms.y,
                           accel_out_ms.x, accel_out_ms.y,
                           accel_raw_ms.x, accel_raw_ms.y,
                           pos_control->get_accel_desired_cmss().xy().length() * 0.01f,
                           roll_in,
                           pos_control->get_pos_error_xy_cm() * 0.01f,
                           pos_control->get_vel_desired_cms().xy().length() * 0.01f,
                           (uint8_t)reason,
                           reason != LTurnExit::NONE);
}
#endif

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeLoiter::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate();

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // the L-turn only runs while flying; abandon it if we leave that state
    if (_ltrn_state != LTurnState::INACTIVE && loiter_state != AltHoldModeState::Flying) {
        ltrn_finish_exit(LTurnExit::NOT_FLYING, ltrn_roll_input());
    }

    // Loiter State Machine
    switch (loiter_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHoldModeState::Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // L-turn: when active it owns the horizontal axes and the heading, and the
        // vertical path below is shared unchanged with stock loiter
        if (ltrn_update()) {
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
#if AP_RANGEFINDER_ENABLED
            copter.surface_tracking.update_surface_offset();
#endif
            pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
            break;
        }

#if AC_PRECLAND_ENABLED
        bool precision_loiter_old_state = _precision_loiter_active;
        if (do_precision_loiter()) {
            precision_loiter_xy();
            _precision_loiter_active = true;
        } else {
            _precision_loiter_active = false;
        }
        if (precision_loiter_old_state && !_precision_loiter_active) {
            // prec loiter was active, not any more, let's init again as user takes control
            loiter_nav->init_target();
        }
        // run loiter controller if we are not doing prec loiter
        if (!_precision_loiter_active) {
            loiter_nav->update();
        }
#else
        loiter_nav->update();
#endif

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

// loiter_exit - called when the vehicle leaves loiter for another mode
void ModeLoiter::exit()
{
    // any mode change aborts an L-turn and returns the borrowed PosControl limits
    if (_ltrn_state != LTurnState::INACTIVE) {
        ltrn_finish_exit(LTurnExit::MODE_CHANGE, ltrn_roll_input());
    }
}

uint32_t ModeLoiter::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeLoiter::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

#endif
