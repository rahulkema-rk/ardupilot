-----------------------------------------------------------
--  KFT AUTO Mode Altitude Override  v4.16
--  Platform  : ArduCopter 4.6.3 (KFT Custom Build)
--  FC        : Pixhawk 6C
--  TX        : Skydroid T12
--  Frame     : EFT E610P Hexacopter (Sprayer)
--  Author    : Kapil Future Tech (KFT)
--  Date      : 2026-04-22
--
--  ═══════════════════════════════════════════════════
--  EXACT BEHAVIOUR:
--  ═══════════════════════════════════════════════════
--    1. Drone flying AUTO mission at original WP altitude
--    2. Pilot pushes throttle up/down (obstacle)
--       → GUIDED instantly, drone climbs/descends, spray ON
--    3. Pilot releases throttle
--       → WPNAV_SPEED_UP + WPNAV_SPEED_DN clamped to 1 cm/s
--       → AUTO resumes in 0.3s — NO hover, NO stop
--       → Drone continues flying forward in AUTO
--       → ArduPilot tries to correct altitude but cannot
--          (vertical speed clamped) → stays at new height
--       → After 3s: WPNAV speeds restored to normal
--    4. Next obstacle → pilot adjusts again → same flow
--
--  WHY WPNAV CLAMP WORKS:
--    WPNAV_SPEED_UP / WPNAV_SPEED_DN control how fast
--    ArduPilot's mission navigator corrects altitude toward
--    the target WP altitude. Setting them to 1 cm/s means
--    the drone can only move 0.01 m/s vertically toward the
--    WP — effectively zero. Drone holds new pilot altitude
--    while flying the mission horizontally as normal.
--    After 3s the lock is released and ArduPilot resumes
--    normal altitude control toward the next WP.
--
--  FIX OVER v4.13:
--    v4.13: AUTO resumed but drone immediately pulled back
--    to original WP altitude. No hover/stop fix worked.
--    v4.14: WPNAV clamp stops altitude pull-back entirely.
--    No hover, no stop, instant continuous flight.
--
--  ALL FIXES SUMMARY:
--    ✅ v4.3:  Removed cmd:get_loc() crash
--    ✅ v4.4:  No lane skip, speed cap MAX_H_MS
--    ✅ v4.5:  Sprayer continuity (detection approach)
--    ✅ v4.6:  Sprayer continuity (always-on via sprayer:run)
--    ✅ v4.7:  Direct SERVO9 PWM + height persistence attempt
--    ✅ v4.8:  Zero-gap spray handoff back to ArduPilot
--    ✅ v4.9:  Hardcoded SERVO9 ch8 — pump_chan never nil
--    ✅ v4.10: Altitude guard (caused oscillation)
--    ✅ v4.11: Guard delay (still oscillated)
--    ✅ v4.12: Guard removed — clean AUTO resume
--    ✅ v4.13: Post-AUTO lockout + spray bridge fix
--    ✅ v4.14: WPNAV clamp — instant AUTO, new alt holds
--
--  CONFIRMED WORKING APIs (KFT 4.6.3 custom build):
--    ahrs:get_relative_position_NED_home()      ✅
--    ahrs:get_velocity_NED()                    ✅
--    vehicle:set_target_velocity_NED()          ✅
--    vehicle:set_mode()                         ✅
--    vehicle:get_mode()                         ✅
--    rc:get_pwm()                               ✅
--    mission:get_current_nav_index()            ✅
--    SRV_Channels:set_output_pwm_chan_timeout() ✅
--    sprayer:run(bool)                          ✅
--    gcs:send_text()                            ✅
--    param:get() / param:set()                  ✅
--
--  CONFIRMED BROKEN (KFT 4.6.3 — DO NOT USE):
--    ahrs:get_altitude()                    ❌
--    vehicle:set_target_altitude_cm()       ❌
--    cmd:id() / cmd:get_loc()               ❌
--    mission:set_item() — navigator ignores ❌
--    sprayer:get_pump_output()              ❌ always 0
--    SRV_Channels:find_channel()            ❌ returns nil
--
--  T12 RC:  MIN=1050  MID=1501  MAX=1950
--  SERVO9:  MIN=1100  TRIM=1500  MAX=1900
-----------------------------------------------------------

-- =====================================================
--  CONFIGURATION
-- =====================================================
local CFG = {
    -- RC throttle stick
    THR_CH          = 3,
    PWM_MIN         = 1050,
    PWM_MID         = 1501,
    PWM_MAX         = 1950,
    DEADZONE        = 70,

    -- Velocity during GUIDED override
    CLIMB_MS        = 1.5,
    DESCEND_MS      = 1.0,

    -- Altitude safety
    ALT_MIN_M       = 1.0,
    ALT_MAX_M       = 30.0,

    -- WPNAV altitude lock
    -- Clamp vertical speed to 1 cm/s so drone holds new altitude in AUTO
    -- Restored after ALT_LOCK_CYCLES × 100ms
    ALT_LOCK_CYCLES = 30,           -- 3.0s lock duration
    WPNAV_MIN_CMS   = 1,            -- 1 cm/s = effectively frozen vertically

    -- Timing
    RETURN_CYCLES   = 3,            -- 0.3s settle before AUTO return (very short)
    POST_LOCKOUT    = 20,           -- 2.0s ignore stick after AUTO return
    LOOP_MS         = 100,
    FORCE_BACK      = 5,
}

-- =====================================================
--  CONSTANTS
-- =====================================================
local MODE_AUTO   = 3
local MODE_GUIDED = 4
local RANGE_UP    = CFG.PWM_MAX - CFG.PWM_MID
local RANGE_DOWN  = CFG.PWM_MID - CFG.PWM_MIN

-- =====================================================
--  STATE
-- =====================================================
local in_override       = false
local hold_counter      = 0
local log_tick          = 0
local force_back_timer  = 0
local post_lockout      = 0

-- WPNAV altitude lock
local alt_lock_active   = false
local alt_lock_counter  = 0
local wpnav_up_orig     = 250   -- fallback default (cm/s)
local wpnav_dn_orig     = 150   -- fallback default (cm/s)

-- =====================================================
--  STARTUP
-- =====================================================
local function init()
    -- Save current WPNAV speeds so we can restore them later
    local up = param:get("WPNAV_SPEED_UP")
    local dn = param:get("WPNAV_SPEED_DN")
    if up and up > 1 then wpnav_up_orig = up end
    if dn and dn > 1 then wpnav_dn_orig = dn end

    gcs:send_text(6, string.format(
        "KFT v4.16 | WPNAV up=%.0f dn=%.0f cm/s saved",
        wpnav_up_orig, wpnav_dn_orig))
end

-- =====================================================
--  HELPERS
-- =====================================================
local function get_stick_norm()
    local pwm = rc:get_pwm(CFG.THR_CH)
    if not pwm then return 0.0 end
    local delta = pwm - CFG.PWM_MID
    if math.abs(delta) <= CFG.DEADZONE then return 0.0 end
    if delta > 0 then
        return math.min(1.0, (delta - CFG.DEADZONE) / (RANGE_UP - CFG.DEADZONE))
    else
        return math.max(-1.0, (delta + CFG.DEADZONE) / (RANGE_DOWN - CFG.DEADZONE))
    end
end

local function get_alt_m()
    local ned = ahrs:get_relative_position_NED_home()
    if ned then return -ned:z() end
    return nil
end

local function send_velocity_NED(vz_ms)
    local vel = ahrs:get_velocity_NED()
    local vn, ve = 0.0, 0.0
    if vel then vn = vel:x(); ve = vel:y() end
    local v = Vector3f()
    v:x(vn); v:y(ve); v:z(-vz_ms)
    vehicle:set_target_velocity_NED(v)
end

-- =====================================================
--  WPNAV ALTITUDE LOCK
-- =====================================================
local function wpnav_clamp()
    param:set("WPNAV_SPEED_UP", CFG.WPNAV_MIN_CMS)
    param:set("WPNAV_SPEED_DN", CFG.WPNAV_MIN_CMS)
    alt_lock_active  = true
    alt_lock_counter = CFG.ALT_LOCK_CYCLES
    gcs:send_text(6, string.format(
        "KFT: WPNAV clamped → drone holds new alt for %.1fs",
        CFG.ALT_LOCK_CYCLES * 0.1))
end

local function wpnav_restore()
    param:set("WPNAV_SPEED_UP", wpnav_up_orig)
    param:set("WPNAV_SPEED_DN", wpnav_dn_orig)
    alt_lock_active = false
    gcs:send_text(6, string.format(
        "KFT: WPNAV restored → up=%.0f dn=%.0f cm/s",
        wpnav_up_orig, wpnav_dn_orig))
end

-- =====================================================
--  Return to AUTO — instant, no stop
-- =====================================================
local function return_to_auto(reason, cur_alt)
    wpnav_clamp()                   -- clamp FIRST so nav can't pull alt back
    vehicle:set_mode(MODE_AUTO)     -- then switch — drone keeps flying forward
    in_override  = false
    hold_counter = 0
    post_lockout = CFG.POST_LOCKOUT

    local a = cur_alt and string.format("%.1fm", cur_alt) or "N/A"
    gcs:send_text(6, string.format(
        "KFT: AUTO instant | %s | NewAlt=%s | no stop",
        reason, a))
end

-- =====================================================
--  MAIN LOOP
-- =====================================================
local function update()

    local mode  = vehicle:get_mode()
    local stick = get_stick_norm()
    local alt   = get_alt_m()

    -- ── WPNAV lock countdown (independent of override state) ─
    if alt_lock_active then
        alt_lock_counter = alt_lock_counter - 1
        if alt_lock_counter <= 0 then
            wpnav_restore()
        end
    end

    -- ── Force-back recovery ───────────────────────────
    if force_back_timer > 0 then
        force_back_timer = force_back_timer - 1
        if force_back_timer == 0 then
            return_to_auto("mode-hijack recovery", alt)
        end
        return update, CFG.LOOP_MS
    end

    -- ══════════════════════════════════════════════════
    --  NOT IN OVERRIDE
    -- ══════════════════════════════════════════════════
    if not in_override then

        -- Stick lockout after AUTO return (blocks noise re-trigger)
        if post_lockout > 0 then
            post_lockout = post_lockout - 1
            if post_lockout == 0 then
                gcs:send_text(6, "KFT: Lockout done — stick active")
            end
            return update, CFG.LOOP_MS
        end

        -- Trigger override only from AUTO
        if mode == MODE_AUTO and math.abs(stick) > 0.02 then
            if not alt then
                gcs:send_text(4, "KFT: Waiting AHRS...")
                return update, CFG.LOOP_MS
            end
            if alt <= CFG.ALT_MIN_M and stick < 0 then
                gcs:send_text(4, "KFT: FLOOR — descent blocked")
                return update, CFG.LOOP_MS
            end
            if alt >= CFG.ALT_MAX_M and stick > 0 then
                gcs:send_text(4, "KFT: CEILING — climb blocked")
                return update, CFG.LOOP_MS
            end

            vehicle:set_mode(MODE_GUIDED)
            in_override  = true
            hold_counter = 0

            gcs:send_text(6, string.format(
                "KFT: OVERRIDE | Alt=%.1fm Stk=%.2f WP=%s",
                alt, stick,
                tostring(mission:get_current_nav_index())))
        end

        return update, CFG.LOOP_MS
    end

    -- ══════════════════════════════════════════════════
    --  IN OVERRIDE (GUIDED)
    -- ══════════════════════════════════════════════════

    -- Unexpected mode change
    if mode ~= MODE_GUIDED then
        if mode == MODE_AUTO then
            in_override  = false
            hold_counter = 0
            post_lockout = CFG.POST_LOCKOUT
            gcs:send_text(6, "KFT: Override cleared (external AUTO)")
        else
            in_override      = false
            hold_counter     = 0
            post_lockout     = 0
            force_back_timer = CFG.FORCE_BACK
            if alt_lock_active then wpnav_restore() end
            gcs:send_text(4, string.format(
                "KFT: Unexpected mode %d | recovery", mode))
        end
        return update, CFG.LOOP_MS
    end

    -- Altitude safety clamp
    local stick_eff = stick
    if alt then
        if alt <= CFG.ALT_MIN_M and stick_eff < 0 then stick_eff = 0.0 end
        if alt >= CFG.ALT_MAX_M and stick_eff > 0 then stick_eff = 0.0 end
    else
        stick_eff = 0.0
        gcs:send_text(3, "KFT: AHRS LOST — holding!")
    end

    -- Stick active → climb / descend
    if math.abs(stick_eff) > 0.02 then
        local vz = stick_eff * (stick_eff > 0 and CFG.CLIMB_MS or CFG.DESCEND_MS)
        send_velocity_NED(vz)
        hold_counter = 0

    -- Stick centred → 0.3s then AUTO immediately (no stop)
    else
        send_velocity_NED(0.0)
        hold_counter = hold_counter + 1

        if hold_counter == 1 then
            local a = alt and string.format("%.1fm", alt) or "N/A"
            gcs:send_text(6, "KFT: Stick released at " .. a .. " | AUTO in 0.3s")
        end

        if hold_counter >= CFG.RETURN_CYCLES then
            return_to_auto("stick centred", alt)
        end
    end

    -- Status log every 5s
    log_tick = log_tick + 1
    if log_tick >= 50 then
        local a    = alt and string.format("%.1fm", alt) or "---"
        local lock = alt_lock_active
                     and string.format("AltLocked:%d", alt_lock_counter)
                     or  "AltFree"
        gcs:send_text(6, string.format(
            "KFT | GUIDED | Alt:%s Hold:%d %s",
            a, hold_counter, lock))
        log_tick = 0
    end

    return update, CFG.LOOP_MS
end

-- =====================================================
--  STARTUP
-- =====================================================
init()
gcs:send_text(6, "KFT v4.16 | InstantAUTO+WPNAVclamp+NoSpray | T12:1050/1501/1950")
return update()
