-- ============================================================
--  KFT | DJI-Style Two-Stick Arming Script for ArduPilot
--  Target  : Cube Orange+ / Pixhawk 6C  ArduCopter V4.6.2
--  File    : dji_stick_arming.lua
--  Place   : SD:/APM/scripts/dji_stick_arming.lua
--  Version : 1.5.0 — native uint32 millis(), no tonumber() needed
-- ============================================================
--
--  RC CALIBRATION : Min=1051  Mid=1501  Max=1951
--  LOW_PWM = 1280  |  HIGH_PWM = 1720
--
--  ARM   (Mode 2) : Left DOWN+RIGHT  + Right DOWN+LEFT  → hold 2s
--  DISARM(Mode 2) : Left DOWN+LEFT   + Right DOWN+RIGHT → hold 2s
--
--  REQUIRED PARAMETERS
--  SCR_ENABLE    = 1
--  ARMING_RUDDER = 0
-- ============================================================

-- ── User settings ────────────────────────────────────────────
local DEBUG_ENABLE  = true    -- set false once working
local DEBUG_RATE_MS = 1000    -- print PWM every 1 second

-- ── Configuration ────────────────────────────────────────────
local LOW_PWM        = 1280
local HIGH_PWM       = 1720
local ARM_HOLD_MS    = 2000
local DISARM_HOLD_MS = 2000
local UPDATE_RATE_MS = 50

local CH_ROLL     = 1
local CH_PITCH    = 2
local CH_THROTTLE = 3
local CH_YAW      = 4

local VERSION = "1.5.0"

-- ── State — stored as plain integers (not uint32) ────────────
-- We use integer 0 as sentinel and compare using uint32 from millis()
local arm_start         = uint32_t(0)
local disarm_start      = uint32_t(0)
local last_notif_arm    = uint32_t(0)
local last_notif_disarm = uint32_t(0)
local last_debug_ms     = uint32_t(0)

local holding_arm       = false
local holding_disarm    = false

-- ── Helpers ──────────────────────────────────────────────────
-- Use millis() uint32 directly — no tonumber() conversion needed.
-- ArduPilot uint32_t supports: +, -, >=, <=, ==, ~=  natively.

local function get_pwm(ch)
  local v = rc:get_pwm(ch)
  if v == nil then return 1501 end
  return v  -- rc:get_pwm() returns plain int on 4.6.x
end

local function is_low(pwm)  return pwm < LOW_PWM  end
local function is_high(pwm) return pwm > HIGH_PWM end

-- ── Debug: live PWM ──────────────────────────────────────────
local function debug_pwm()
  if not DEBUG_ENABLE then return end
  local now = millis()
  if (now - last_debug_ms) >= DEBUG_RATE_MS then
    local thr  = get_pwm(CH_THROTTLE)
    local yaw  = get_pwm(CH_YAW)
    local roll = get_pwm(CH_ROLL)
    local pit  = get_pwm(CH_PITCH)

    local ts = is_low(thr)   and "L" or "-"
    local ys = is_high(yaw)  and "H" or (is_low(yaw)  and "L" or "-")
    local rs = is_low(roll)  and "L" or (is_high(roll) and "H" or "-")
    local ps = is_low(pit)   and "L" or "-"

  --  gcs:send_text(6, string.format(
  -- "PWM THR:%d(%s) YAW:%d(%s) ROL:%d(%s) PIT:%d(%s)",
  --thr, ts, yaw, ys, roll, rs, pit, ps))

    last_debug_ms = now
  end
end

-- ── Gestures ─────────────────────────────────────────────────
--  ARM   : THR=L, YAW=H, ROL=L, PIT=L
--  DISARM: THR=L, YAW=L, ROL=H, PIT=L
local function arm_gesture()
  return is_low (get_pwm(CH_THROTTLE)) and
         is_high(get_pwm(CH_YAW))      and
         is_low (get_pwm(CH_ROLL))     and
         is_high (get_pwm(CH_PITCH))
end

local function disarm_gesture()
  return is_low (get_pwm(CH_THROTTLE)) and
         is_low (get_pwm(CH_YAW))      and
         is_high(get_pwm(CH_ROLL))     and
         is_high (get_pwm(CH_PITCH))
end

-- ── Progress notification ─────────────────────────────────────
-- elapsed and total are uint32 — convert to float only for division
local function notify(label, elapsed, total, last_ref)
  local now = millis()
  if (now - last_ref) >= 500 then
    local e = elapsed:tofloat()
    local t = total
    local pct = math.floor((e / t) * 100.0)
    gcs:send_text(5, string.format("KFT | %s... %d%%", label, pct))
    return now
  end
  return last_ref
end

-- ── Main loop ────────────────────────────────────────────────
local function update()
  local now = millis()

  debug_pwm()

  -- ── ARM ──────────────────────────────────────────────────
  if arm_gesture() then
    if not holding_arm then
      arm_start      = now
      holding_arm    = true
      holding_disarm = false
      --gcs:send_text(6, "KFT | ARM gesture detected - hold 2s")
    else
      local elapsed = now - arm_start
      if elapsed >= ARM_HOLD_MS then
        if not arming:is_armed() then
          if is_low(get_pwm(CH_THROTTLE)) then
            if arming:arm() then
              gcs:send_text(3, "KFT | *** ARMED ***")
            else
              gcs:send_text(4, "KFT | Arm FAILED - fix pre-arm errors")
            end
          else
            gcs:send_text(4, "KFT | Arm blocked - throttle not LOW")
          end
        end
        holding_arm = false
      else
        last_notif_arm = notify("ARMING", elapsed, ARM_HOLD_MS, last_notif_arm)
      end
    end
  else
    holding_arm = false
  end

  -- ── DISARM ─────────────────────────────────────────────────
  if disarm_gesture() then
    if not holding_disarm then
      disarm_start   = now
      holding_disarm = true
      holding_arm    = false
    --gcs:send_text(6, "KFT | DISARM gesture detected - hold 2s")
    else
      local elapsed = now - disarm_start
      if elapsed >= DISARM_HOLD_MS then
        if arming:is_armed() then
          if arming:disarm() then
            gcs:send_text(3, "KFT | *** DISARMED ***")
          else
            gcs:send_text(4, "KFT | Disarm FAILED")
          end
        end
        holding_disarm = false
      else
        last_notif_disarm = notify("DISARMING", elapsed, DISARM_HOLD_MS, last_notif_disarm)
      end
    end
  else
    holding_disarm = false
  end

  return update, UPDATE_RATE_MS
end

-- ── Boot ─────────────────────────────────────────────────────
gcs:send_text(3, string.format(
  "KFT Arm Script v%s | LOW<%d HIGH>%d | DEBUG:%s",
  VERSION, LOW_PWM, HIGH_PWM,
  DEBUG_ENABLE and "ON" or "OFF"))

return update, UPDATE_RATE_MS
