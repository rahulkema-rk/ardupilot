-- sharp_turn.lua  (KFT rev v4.1 Sliding-Stick + LTN_ params, ArduPilot Copter 4.6.x)
-- Sharp robotic "L" turn for multirotors flown in LOITER.
-- Supports both Forward and Backward approaches.
--
-- v4.0: Tuning variables migrated to live-tunable ArduPilot parameters (LTN_ prefix).
-- v4.1: "Sliding stick" trigger (roll-dominance instead of pitch-centered). Pitch-based
--       aborts removed -> the ONLY cancel is centering the roll stick. New tunable
--       LTN_ROLL_DOM sets how much roll must exceed pitch to arm the maneuver.
--       State machine structure, kinematics and slew math are otherwise UNCHANGED.

----------------------------------------------------------------------
-- FIXED CONFIG (not tunable in GCS)
----------------------------------------------------------------------
local LOITER_MODE = 5
local GUIDED_MODE = 4

local UPDATE_RATE_MS = 50     -- 20 Hz loop period (left as a fixed constant)

-- RCMAP_ROLL / RCMAP_PITCH output channels (defaults 1 and 2).
local ROLL_CHAN  = 1
local PITCH_CHAN = 2

local ENABLE_AUX_FN = nil

----------------------------------------------------------------------
-- PARAMETER TABLE  (live-tunable from Mission Planner)
----------------------------------------------------------------------
-- PARAM_TABLE_KEY:
--   * An integer 0-200 that uniquely identifies THIS script's parameter
--     table inside the flight controller. It is NOT seen by the pilot;
--     it is only used internally to register the table.
--   * It MUST be unique across every Lua script you run at the same time.
--     If two scripts share a key, add_table() on the second one fails and
--     that script stops. If you add other KFT scripts later, give each its
--     own key (e.g. 78, 79, 80 ...).
--   * Changing the key later does NOT rename the LTN_ parameters; the names
--     come from the PREFIX below, not the key.
local PARAM_TABLE_KEY    = 78
local PARAM_TABLE_PREFIX = "LTN_"   -- prefix shown to the pilot (keep total name <= 16 chars)

-- Register the table: (key, prefix, number_of_params). We declare 7 params.
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 7),
       "LTN: could not add param table (key in use?)")

-- Helper: register one param (1-based index, must be <= count above) and
-- return a bound Parameter object so we can call :get() cheaply each loop.
local function bind_add_param(suffix, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, suffix, default_value),
           string.format("LTN: could not add param %s", suffix))
    local p = Parameter()
    assert(p:init(PARAM_TABLE_PREFIX .. suffix),
           string.format("LTN: could not bind param %s", suffix))
    return p
end

-- Bind the six tunables. The full name the pilot sees = PREFIX .. suffix.
-- All names stay <= 16 chars (ArduPilot hard limit).
--   LTN_LAT_SPD   <- MAX_LATERAL_SPEED  (m/s at full roll stick)
--   LTN_FWD_STOP  <- FWD_STOP_SPEED     (m/s, fwd/back speed treated as "stopped")
--   LTN_DEADZONE  <- STICK_DEADZONE     (normalized, stick considered centered)
--   LTN_ROLL_TRIG <- ROLL_TRIGGER       (normalized roll needed to arm the maneuver)
--   LTN_MIN_ENTRY <- MIN_ENTRY_FWD      (m/s, min absolute speed to trigger)
--   LTN_LAT_ACCEL <- LATERAL_ACCEL_RATE (m/s^2, lateral roll-in smoothness; lower = softer)
--   LTN_ROLL_DOM  <- ROLL_DOMINANCE     (normalized; how much roll must exceed pitch to arm
--                                        the "sliding stick" trigger. Higher = more deliberate
--                                        slide required, fewer accidental L-turns from diagonals)
local p_lat_spd   = bind_add_param("LAT_SPD",   1, 8.0)
local p_fwd_stop  = bind_add_param("FWD_STOP",  2, 5.15)
local p_deadzone  = bind_add_param("DEADZONE",  3, 0.35)
local p_roll_trig = bind_add_param("ROLL_TRIG", 4, 0.60)
local p_min_entry = bind_add_param("MIN_ENTRY", 5, 1.5)
local p_lat_accel = bind_add_param("LAT_ACCEL", 6, 2.45)
local p_roll_dom  = bind_add_param("ROLL_DOM",  7, 0.30)

----------------------------------------------------------------------
-- States
----------------------------------------------------------------------
local S_IDLE, S_BRAKING, S_LATERAL = 0, 1, 2
local state = S_IDLE

local locked_yaw_rad = 0.0
local locked_yaw_deg = 0.0
local current_vy_body = 0.0 

local vel_cmd    = Vector3f()
local zero_accel = Vector3f()

----------------------------------------------------------------------
-- HELPERS
----------------------------------------------------------------------
local function feature_enabled()
    if not ENABLE_AUX_FN then return true end
    return rc:get_aux_cached(ENABLE_AUX_FN) == 2
end

local function body_right_to_ne(vy_body, yaw_rad)
    return -vy_body * math.sin(yaw_rad), vy_body * math.cos(yaw_rad)
end

local function forward_speed(vn, ve, yaw_rad)
    return vn * math.cos(yaw_rad) + ve * math.sin(yaw_rad)
end

local function cmd_vel_locked_yaw(vn, ve)
    vel_cmd:x(vn); vel_cmd:y(ve); vel_cmd:z(0)
    vehicle:set_target_velaccel_NED(
        vel_cmd, zero_accel,
        true,  locked_yaw_deg,
        false, 0.0,
        false)
end

local function exit_to_loiter(sev, msg)
    vehicle:set_mode(LOITER_MODE)
    state = S_IDLE
    if msg then gcs:send_text(sev, msg) end
end

----------------------------------------------------------------------
-- MAIN
----------------------------------------------------------------------
function update()
    -- Fetch live-tunable params each loop so GCS edits take effect immediately.
    -- These locals reuse the original variable names, so the logic below is
    -- byte-for-byte identical to the hardcoded version.
    local MAX_LATERAL_SPEED  = p_lat_spd:get()
    local FWD_STOP_SPEED     = p_fwd_stop:get()
    local STICK_DEADZONE     = p_deadzone:get()
    local ROLL_TRIGGER       = p_roll_trig:get()
    local MIN_ENTRY_FWD      = p_min_entry:get()
    local LATERAL_ACCEL_RATE = p_lat_accel:get()
    local ROLL_DOMINANCE     = p_roll_dom:get()

    local mode = vehicle:get_mode()

    if vehicle:has_ekf_failsafed() then
        if state ~= S_IDLE then
            state = S_IDLE
            gcs:send_text(4, "L-Turn: abort (EKF failsafe)")
        end
        return update, UPDATE_RATE_MS
    end

    if not rc:has_valid_input() then
        if state ~= S_IDLE then
            exit_to_loiter(4, "L-Turn: abort (RC invalid)")
        end
        return update, UPDATE_RATE_MS
    end

    local rollc  = rc:get_channel(ROLL_CHAN)
    local pitchc = rc:get_channel(PITCH_CHAN)
    local vned   = ahrs:get_velocity_NED()
    if not rollc or not pitchc or not vned then
        return update, UPDATE_RATE_MS
    end

    local roll_in  = rollc:norm_input_dz()
    local pitch_in = pitchc:norm_input_dz()
    local yaw_rad  = ahrs:get_yaw()
    local vn, ve   = vned:x(), vned:y()

    ------------------------------------------------------------------
    if state == S_IDLE then
        -- "Sliding stick" trigger: no longer requires pitch to be centered.
        -- Instead, roll must DOMINATE pitch (roll > pitch + ROLL_DOMINANCE) so the
        -- pilot can sweep the thumb from forward into a roll without snapping to center.
        -- Note: at high forward pitch this is mathematically un-armable (good), so the
        -- maneuver only fires once pitch has bled down enough for roll to win.
        if mode == LOITER_MODE
           and feature_enabled()
           and vehicle:get_likely_flying()
           and math.abs(roll_in)  > ROLL_TRIGGER
           and math.abs(roll_in)  > (math.abs(pitch_in) + ROLL_DOMINANCE)
           and math.abs(forward_speed(vn, ve, yaw_rad)) > MIN_ENTRY_FWD then

            if vehicle:set_mode(GUIDED_MODE) then
                locked_yaw_rad = yaw_rad
                locked_yaw_deg = math.deg(yaw_rad)
                state = S_BRAKING
                current_vy_body = 0.0 
                cmd_vel_locked_yaw(0, 0)
                gcs:send_text(6, "L-Turn: braking")
            end
        end

    ------------------------------------------------------------------
    elseif state == S_BRAKING then
        if mode ~= GUIDED_MODE then
            state = S_IDLE
            return update, UPDATE_RATE_MS
        end
        -- Pitch abort REMOVED: during a slide the thumb is still on pitch, so the
        -- only deliberate cancel is centering the roll stick (below).
        if not feature_enabled() then
            exit_to_loiter(4, "L-Turn: abort")
            return update, UPDATE_RATE_MS
        end
        if math.abs(roll_in) < STICK_DEADZONE then
            exit_to_loiter(6, "L-Turn: cancelled")
            return update, UPDATE_RATE_MS
        end

        cmd_vel_locked_yaw(0, 0)

        -- MODIFIED: Use math.abs() to check if backward momentum is dumped
        if math.abs(forward_speed(vn, ve, locked_yaw_rad)) < FWD_STOP_SPEED then
            state = S_LATERAL
            gcs:send_text(6, "L-Turn: lateral")
        end

    ------------------------------------------------------------------
    elseif state == S_LATERAL then
        if mode ~= GUIDED_MODE then
            state = S_IDLE
            return update, UPDATE_RATE_MS
        end
        -- Pitch abort REMOVED here too: releasing the roll stick is the exit.
        if not feature_enabled() then
            exit_to_loiter(4, "L-Turn: abort")
            return update, UPDATE_RATE_MS
        end
        if math.abs(roll_in) < STICK_DEADZONE then
            exit_to_loiter(6, "L-Turn: complete")
            return update, UPDATE_RATE_MS
        end

        local target_vy_body = roll_in * MAX_LATERAL_SPEED
        local max_step = LATERAL_ACCEL_RATE * (UPDATE_RATE_MS / 1000.0)
        
        local error = target_vy_body - current_vy_body
        if math.abs(error) <= max_step then
            current_vy_body = target_vy_body 
        elseif error > 0 then
            current_vy_body = current_vy_body + max_step 
        else
            current_vy_body = current_vy_body - max_step 
        end

        local tn, te  = body_right_to_ne(current_vy_body, locked_yaw_rad)
        cmd_vel_locked_yaw(tn, te)
    end

    return update, UPDATE_RATE_MS
end

gcs:send_text(6, "Sharp Turn script loaded (Bidirectional + LTN_ params)")
return update, 10003
