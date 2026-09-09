-- ================================================================
-- KFT-LUA-LED-001  |  JIYI K++ Style RGB LED Status Indicator
-- ================================================================
-- Platform : ArduCopter 4.6.x  |  Pixhawk 6C
-- Hardware : WS2812B NeoPixel LED on AUX1
-- Author   : Kapil Future Tech (KFT)
-- Version  : 1.0  |  2025
-- ----------------------------------------------------------------
-- REQUIRED PARAMETERS:
--   SERVO13_FUNCTION = 96   (Script3 on AUX5)
--   SCR_ENABLE       = 1
--   SCR_VM_I_COUNT   = 50000
--
-- OPTIONAL (prevents ArduPilot built-in notify from conflicting):
--   NTF_LED_TYPES    = 0    (disable all built-in LED patterns)
--   NTF_LED_LEN      = 1    (number of NeoPixel LEDs in chain)
-- ----------------------------------------------------------------
-- LED STATE MAP  (JIYI K++ style)
--
--   RED slow blink    → Disarmed, no GPS fix
--   CYAN blink        → Disarmed, GPS acquiring (2D fix)
--   GREEN slow blink  → Disarmed, GPS ready (3D+)
--   MAGENTA fast      → No valid RC signal
--   ORANGE solid      → Armed, no GPS
--   YELLOW blink      → Armed, 2D GPS only
--   GREEN heartbeat   → Armed, GPS locked (3D+)  ← JIYI double-blink
--   RED/ORANGE alt    → Low battery warning
--   RED fast blink    → Critical battery
-- ================================================================

-- ── USER CONFIGURATION ─────────────────────────────────────────
local CFG = {
  channel   = 2,      -- serialLED channel: Script3 = index 2 (SERVO13_FUNCTION=96 on AUX5)
  num_leds  = 5,      -- Number of LEDs in chain (Zerodrag V1.0 = 5)
  s_cells   = 6,      -- Battery cell count (4S=4, 6S=6, adjust to your pack)
  cell_low  = 3.55,   -- Per-cell low warning threshold (V)
  cell_crit = 3.40,   -- Per-cell critical threshold    (V)
  loop_hz   = 20,     -- LED update rate (Hz)
}
-- ───────────────────────────────────────────────────────────────

-- Derived battery thresholds
local BATT_LOW_V  = CFG.cell_low  * CFG.s_cells
local BATT_CRIT_V = CFG.cell_crit * CFG.s_cells

-- GPS fix type thresholds (ArduPilot gps:status() values)
local GPS_FIX_2D = 2   -- GPS_OK_FIX_2D
local GPS_FIX_3D = 3   -- GPS_OK_FIX_3D (3D, DGPS, RTK float/fixed all >= 3)

-- Blink half-periods (ms) — time for one ON or one OFF phase
local T_FAST = 100
local T_MED  = 250
local T_SLOW = 600

-- RGB colour table  {R, G, B}
local C = {
  OFF     = {  0,   0,   0},
  RED     = {255,   0,   0},
  GREEN   = {  0, 220,   0},
  YELLOW  = {255, 160,   0},
  ORANGE  = {255,  55,   0},
  CYAN    = {  0, 180, 220},
  MAGENTA = {200,   0, 220},
}

-- Runtime state
local blink_phase = false
local blink_timer = millis()

-- Heartbeat sub-state (0..3): ON / short-off / ON / long-off
local beat_step  = 0
local beat_timer = millis()
local BEAT_MS    = {55, 80, 55, 810}   -- ms per step (total ~1 s cycle)

-- ── LED DRIVER ─────────────────────────────────────────────────

local function led_set(r, g, b)
  for i = 0, CFG.num_leds - 1 do               -- set all LEDs individually (AC 4.6.3 has no broadcast index)
    serialLED:set_RGB(CFG.channel, i, r, g, b)
  end
  serialLED:send(CFG.channel)
end

-- Symmetric blink: colour ↔ off
local function led_blink(color, period_ms)
  local now = millis()
  if (now - blink_timer):tofloat() >= period_ms then
    blink_timer = now
    blink_phase = not blink_phase
  end
  if blink_phase then
    led_set(color[1], color[2], color[3])
  else
    led_set(0, 0, 0)
  end
end

-- Alternating blink: colour A ↔ colour B
local function led_alt(ca, cb, period_ms)
  local now = millis()
  if (now - blink_timer):tofloat() >= period_ms then
    blink_timer = now
    blink_phase = not blink_phase
  end
  -- Note: Lua treats 0 as truthy, so 'and/or' ternary is safe for RGB ints
  led_set(
    blink_phase and ca[1] or cb[1],
    blink_phase and ca[2] or cb[2],
    blink_phase and ca[3] or cb[3]
  )
end

-- JIYI K++ double-blink heartbeat pattern (Armed + GPS locked)
-- Pattern: ON─off─ON─────── (two quick pulses per ~1 s cycle)
local function led_heartbeat(color)
  local now = millis()
  if (now - beat_timer):tofloat() >= BEAT_MS[beat_step + 1] then
    beat_timer = now
    beat_step  = (beat_step + 1) % 4
  end
  -- Steps 0 and 2 = ON, steps 1 and 3 = OFF
  if beat_step == 0 or beat_step == 2 then
    led_set(color[1], color[2], color[3])
  else
    led_set(0, 0, 0)
  end
end

-- ── SYSTEM QUERIES ─────────────────────────────────────────────

local function batt_voltage()
  local v = battery:voltage(0)
  return (v ~= nil and v > 0) and v or 99.9   -- 99.9 = no battery monitor
end

local function rc_valid()
  local pwm = rc:get_pwm(1)
  return pwm ~= nil and pwm > 900 and pwm < 2200
end

-- ── MAIN UPDATE LOOP ───────────────────────────────────────────

local function update()
  local armed   = arming:is_armed()
  local gps_fix = gps:status(0)
  local batt_v  = batt_voltage()
  -- enforce required params
  param:set("NTF_LED_TYPES", 0)
  param:set("SCR_ENABLE", 1)

  -- ── PRIORITY 1: Critical battery (override everything) ──────
  if batt_v < BATT_CRIT_V then
    led_blink(C.RED, T_FAST)
    return
  end

  -- ── PRIORITY 2: Low battery warning ─────────────────────────
  if batt_v < BATT_LOW_V then
    led_alt(C.RED, C.ORANGE, T_MED)
    return
  end

  -- ── PRIORITY 3: Armed states ─────────────────────────────────
  if armed then
    if gps_fix >= GPS_FIX_3D then
      led_heartbeat(C.GREEN)                              -- JIYI heartbeat
    elseif gps_fix == GPS_FIX_2D then
      led_blink(C.YELLOW, T_MED)                         -- armed, 2D only
    else
      led_set(C.ORANGE[1], C.ORANGE[2], C.ORANGE[3])     -- armed, no GPS
    end
    return
  end

  -- ── PRIORITY 4: Disarmed states ──────────────────────────────
  if not rc_valid() then
    led_blink(C.MAGENTA, T_FAST)                         -- RC lost / not bound
    return
  end

  if gps_fix >= GPS_FIX_3D then
    led_blink(C.GREEN, T_SLOW)                           -- ready to arm
  elseif gps_fix == GPS_FIX_2D then
    led_blink(C.CYAN, T_MED)                             -- GPS acquiring
  else
    led_blink(C.RED, T_SLOW)                             -- no GPS
  end
end

-- ── INIT ───────────────────────────────────────────────────────

-- NOTE: set_num_neopixels does NOT exist in AC 4.6.3.
-- LED count is set via NTF_LED_LEN parameter (set to 1).
-- No Lua init call needed — serialLED is ready immediately.
local _startup = 5

local function _update_wrapper()
  if _startup > 0 then
    _startup = _startup - 1
    return
  end
  update()
end

-- Schedule: return wrapper + interval in ms
return _update_wrapper, math.floor(1000 / CFG.loop_hz)
