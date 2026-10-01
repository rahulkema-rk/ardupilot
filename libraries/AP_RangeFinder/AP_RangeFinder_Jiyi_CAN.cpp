/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Jiyi 24GHz FMCW CAN radar backend -- native port of jiyi_radar.lua.

  FRAME FORMAT (verified against a live bus capture, 79 frames, 100% checksum):

    byte 0..3   header, constant per sensor
                  ID 214 (terrain/down) : EA 2D 04 00
                  ID 220 (obstacle/fwd) : D3 3F 04 01
    byte 4..5   distance, uint16 BIG-ENDIAN, centimetres
    byte 6      signal strength (0..255)
    byte 7      checksum
                  ID 214 : sum(bytes 4..6) & 0xFF
                  ID 220 : sum(bytes 0..6) & 0xFF
    distance 0  = NO TARGET. The radar still transmits at ~20 Hz -- a frame
                  carrying 0 means "alive, nothing detected", NOT silence.

  ---------------------------------------------------------------------------
  THINGS THAT MUST NOT BE "TIDIED UP" -- each one cost days to find:

  1. "NO TARGET" IS REPORTED, NOT IGNORED.
     For the forward radar, nothing-detected is the normal state. Staying
     silent lets RNGFND2 age out to NoData, which drags PRX1 to NoData
     (200 ms) and fails prearm. So a clear is actively reported, in range,
     keeping the sector Good.

  2. THE TERRAIN SENSOR IS NEVER GIVEN A SYNTHETIC IN-RANGE DISTANCE.
     It is downward-facing. A fabricated "clear" would tell surface tracking
     it has metres of clearance it does not have. When it has no usable
     return it reports OUT-OF-RANGE-HIGH: fresh, but !Good, so nothing uses
     it. Only the forward sensor ever gets a synthetic in-range value.

  3. ONE DROPPED OR WEAK FRAME MUST NOT WIPE THE FILTER HISTORY.
     At long range returns are intermittent (a tree gives ~70% valid frames
     at 15 m); wiping on every miss means the window never fills. History is
     only cleared after STALE_MS with no usable reading.

  And the safety gate underneath all of it: synthetic reports are published
  ONLY while a sensor has proved liveness (a frame that passed header and
  checksum within SENSOR_ALIVE_MS). A radar that is unplugged, unpowered or
  off the bus gets nothing, falls to NoData, and fails prearm. A dead sensor
  must never be made to look healthy.

  The Lua driver's late CAN acquisition (CAN_ACQUIRE_AT_MS) has no
  counterpart here. It worked around ScriptingCANSensor being constructed on
  the first CAN:get_device() call with no retry. This backend's CANSensor is
  registered from RangeFinder::init(), inside init_ardupilot(), which
  AP_Vehicle::setup() runs after AP_CANManager::init() has configured the
  interfaces; and CANSensor::loop() does not process frames until system
  init has completed.
  ---------------------------------------------------------------------------

  Migration from jiyi_radar.lua:
    RNGFNDn_TYPE 36          -> 120 (this backend)
    CAN_Dx_PROTOCOL 10       -> 14 (RadarCAN)
    rf_instance binding      -> RNGFNDn_RECV_ID  214 terrain / 220 obstacle
    OBSTACLE_MIN_STRENGTH    -> RNGFNDn_SNR_MIN  role default: obstacle 25, terrain 0
    OBSTACLE_MIN_M           -> RNGFNDn_JY_FLR
    OBSTACLE_MEDIAN_N        -> RNGFNDn_JY_MEDN
    CH_OBSTACLE / CH_TERRAIN -> RNGFNDn_JY_ENCH  role default: obstacle 8, terrain 9
    IDLE_TERRAIN_STANDS_DOWN,
    IDLE_OBSTACLE_REPORTS_CLEAR -> RNGFNDn_JY_IDLE role default: terrain 1, obstacle 0
    MODE = "diag"            -> RNGFNDn_JY_DIAG
    HEARTBEAT_MS             -> JIYI dataflash message, always written at 1 Hz
    STALE_MS, MIN_SAMPLES, SENSOR_ALIVE_MS, RC_ENABLE_US -> constants below
    BUFFER_LEN, CAN_ACQUIRE_AT_MS, CAN_RETRY_MS, BIND_* -> not applicable
    boot-time "JIYI! ..." faults -> prearm failures, see prearm_healthy()
 */

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_JIYI_CAN_ENABLED

#include "AP_RangeFinder_Jiyi_CAN.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL& hal;

// Filter behaviour. These are timed from frame arrival on the CAN thread, so
// they do not depend on how often RangeFinder::update() runs (20 Hz on Copter).
static constexpr uint32_t STALE_MS = 400;           // no valid target this long -> genuinely no target
static constexpr uint8_t MIN_SAMPLES = 3;           // report once we have this many

// Nothing is synthesised for a sensor that has not produced a decodable frame
// this recently. See the safety gate note above.
static constexpr uint32_t SENSOR_ALIVE_MS = 500;

// Once publishing stops, the last value is held this long before NoData. This
// is AP_RANGEFINDER_LUA_TIMEOUT_MS, which the Lua driver's reports were subject to.
static constexpr uint32_t PUBLISH_TIMEOUT_MS = 500;

static constexpr int16_t RC_ENABLE_US = 1500;       // midpoint of the 1051/1951 calibration
static constexpr uint8_t PRX_TYPE_RANGEFINDER = 4;

const AP_RangeFinder_Jiyi_CAN::DecodeConfig AP_RangeFinder_Jiyi_CAN::DECODE[2] {
    {
        ID_TERRAIN, "T", "terrain",
        { 0xEA, 0x2D, 0x04, 0x00 },
        4, 6,               // chk_lo, chk_hi
        4, 5,               // dist_hi, dist_lo
        0.01f,              // scale_to_m
        0.02f, 50.0f,       // sane_min_m, sane_max_m
        0,                  // near_rank: plain median
        false,              // synth_mode "out_of_range": never a fabricated in-range distance
        false,              // clamp_to_window
        9,                  // enable_ch
        true,               // idle_stand_down: IDLE_TERRAIN_STANDS_DOWN
        0,                  // min_strength
        0,                  // expect_option: must not share a switch with an aux function
    },
    {
        ID_OBSTACLE, "O", "obstacle",
        { 0xD3, 0x3F, 0x04, 0x01 },
        0, 6,               // chk_lo, chk_hi
        4, 5,               // dist_hi, dist_lo
        0.01f,              // scale_to_m
        0.02f, 50.0f,       // sane_min_m, sane_max_m
        2,                  // near_rank: 2nd smallest, see select_filtered()
        true,               // synth_mode "clear": feeds PRX1, needs an in-range Good value
        true,               // clamp_to_window
        8,                  // enable_ch
        false,              // idle_stand_down: IDLE_OBSTACLE_REPORTS_CLEAR
        25,                 // min_strength: OBSTACLE_MIN_STRENGTH
        40,                 // expect_option: Proximity Avoidance Enable
    },
};

AP_RangeFinder_Jiyi_CAN::CachedParam AP_RangeFinder_Jiyi_CAN::prx_params[PRX_PARAM_COUNT] {
    { "PRX1_TYPE",     nullptr, AP_PARAM_NONE, false },
    { "PRX1_MIN",      nullptr, AP_PARAM_NONE, false },
    { "PRX1_MAX",      nullptr, AP_PARAM_NONE, false },
    { "PRX_ALT_MIN",   nullptr, AP_PARAM_NONE, false },
    { "PRX_IGN_GND",   nullptr, AP_PARAM_NONE, false },
    { "PRX1_IGN_WID1", nullptr, AP_PARAM_NONE, false },
    { "PRX1_IGN_WID2", nullptr, AP_PARAM_NONE, false },
    { "PRX1_IGN_WID3", nullptr, AP_PARAM_NONE, false },
    { "PRX1_IGN_WID4", nullptr, AP_PARAM_NONE, false },
};

const AP_Param::GroupInfo AP_RangeFinder_Jiyi_CAN::var_info[] = {

    // RECV_ID and SNR_MIN are documented in AP_RangeFinder_Backend_CAN.cpp.
    // For this backend RECV_ID selects the radar (214 terrain, 220 obstacle;
    // anything else fails prearm) and SNR_MIN gates on frame byte 6.
    AP_GROUPINFO("RECV_ID", 10, AP_RangeFinder_Jiyi_CAN, receive_id, 0),
    AP_GROUPINFO("SNR_MIN", 11, AP_RangeFinder_Jiyi_CAN, snr_min, 0),

    // @Param: JY_FLR
    // @DisplayName: Jiyi radar ghost floor
    // @Description: Readings closer than this are rejected as ghosts. 0 disables. This blinds the radar below this distance.
    // @Units: m
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("JY_FLR", 12, AP_RangeFinder_Jiyi_CAN, floor_m, 0),

    // @Param: JY_MEDN
    // @DisplayName: Jiyi radar filter width
    // @Description: Number of recent readings the filter selects from. Wider is smoother but laggier. The obstacle radar reports the 2nd smallest, the terrain radar the median.
    // @Range: 3 15
    // @User: Advanced
    AP_GROUPINFO("JY_MEDN", 13, AP_RangeFinder_Jiyi_CAN, median_n, 5),

    // @Param: JY_ENCH
    // @DisplayName: Jiyi radar enable channel
    // @Description: RC channel of this radar's pilot switch. High (1500us or more) or no signal means enabled. 0 means no switch. Defaults to 8 for the obstacle radar and 9 for the terrain radar. The obstacle channel should have RCn_OPTION=40; the terrain channel must have RCn_OPTION=0.
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("JY_ENCH", 14, AP_RangeFinder_Jiyi_CAN, enable_ch, 0),

    // @Param: JY_IDLE
    // @DisplayName: Jiyi radar switch-low policy
    // @Description: What the radar reports while its enable switch is low. 0: real distances. 1: stand down -- the terrain radar reports out-of-range-high so surface tracking stops using it; the obstacle radar reports all clear, which makes PRX1 and the logs show clear when it may not be. Defaults to 1 for terrain and 0 for obstacle.
    // @Values: 0:Report real distances,1:Stand down
    // @User: Standard
    AP_GROUPINFO("JY_IDLE", 15, AP_RangeFinder_Jiyi_CAN, idle_stand_down, 0),

    // @Param: JY_DIAG
    // @DisplayName: Jiyi radar diagnostics
    // @Description: Send a decode summary for this radar to the GCS every second, for bench testing.
    // @Values: 0:Off,1:On
    // @User: Advanced
    AP_GROUPINFO("JY_DIAG", 16, AP_RangeFinder_Jiyi_CAN, diag, 0),

    AP_GROUPEND
};

AP_RangeFinder_Jiyi_CAN::AP_RangeFinder_Jiyi_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend_CAN(_state, _params, AP_CAN::Protocol::RadarCAN, "jiyi")
{
    // replace the base class parameter table with ours
    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
}

const AP_RangeFinder_Jiyi_CAN::DecodeConfig *AP_RangeFinder_Jiyi_CAN::get_config() const
{
    for (const auto &cfg : DECODE) {
        if (receive_id.get() == int32_t(cfg.can_id)) {
            return &cfg;
        }
    }
    return nullptr;
}

AP_RangeFinder_Jiyi_CAN::Reason AP_RangeFinder_Jiyi_CAN::decode(const DecodeConfig &cfg, const AP_HAL::CANFrame &frame,
                                                                float floor_m, int32_t min_strength,
                                                                float &dist_m, uint8_t &strength)
{
    if (frame.dlc < 8) {
        return Reason::DLC;
    }
    for (uint8_t i = 0; i < ARRAY_SIZE(cfg.header); i++) {
        if (frame.data[i] != cfg.header[i]) {
            return Reason::HDR;
        }
    }
    uint16_t sum = 0;
    for (uint8_t i = cfg.chk_lo; i <= cfg.chk_hi; i++) {
        sum += frame.data[i];
    }
    if ((sum & 0xFF) != frame.data[7]) {
        return Reason::CHK;
    }
    strength = frame.data[6];

    const uint16_t raw = (uint16_t(frame.data[cfg.dist_hi]) << 8) | frame.data[cfg.dist_lo];
    if (raw == 0) {
        return Reason::NOTGT;
    }
    const float d = raw * cfg.scale_to_m;
    if (d < cfg.sane_min_m || d > cfg.sane_max_m) {
        return Reason::RANGE;
    }
    if (floor_m > 0 && d < floor_m) {
        return Reason::FLOOR;
    }
    if (min_strength > 0 && strength < min_strength) {
        return Reason::LOW_STRENGTH;
    }
    dist_m = d;
    return Reason::OK;
}

float AP_RangeFinder_Jiyi_CAN::select_filtered(const float *values, uint8_t len, uint8_t near_rank)
{
    float t[MEDIAN_N_MAX];
    len = MIN(len, MEDIAN_N_MAX);
    memcpy(t, values, len * sizeof(t[0]));
    for (uint8_t i = 1; i < len; i++) {
        const float v = t[i];
        uint8_t j = i;
        for (; j > 0 && t[j-1] > v; j--) {
            t[j] = t[j-1];
        }
        t[j] = v;
    }
    if (near_rank > 0) {
        // Obstacle: 2nd smallest of 5. One spurious near spike is still
        // rejected, but a real closing obstacle is reported as soon as 2 of 5
        // frames see it. For avoidance the nearest credible return must win:
        // a plain median can hide a near tree behind older far samples for
        // ~200 ms, which is 1.6 m at 8 m/s.
        return t[MIN(near_rank, len) - 1];
    }
    return t[(len + 1) / 2 - 1];
}

// push a usable reading into the history; returns true with the filtered
// value once the window holds MIN_SAMPLES. Caller holds _sem.
bool AP_RangeFinder_Jiyi_CAN::median_push(const DecodeConfig &cfg, float d, float &result)
{
    const uint8_t n = constrain_int16(median_n.get(), MIN_SAMPLES, MEDIAN_N_MAX);
    // drop the oldest to make room; a loop so a JY_MEDN reduced in flight takes effect
    while (hist_len >= n) {
        memmove(&hist[0], &hist[1], (hist_len - 1) * sizeof(hist[0]));
        hist_len--;
    }
    hist[hist_len++] = d;
    if (hist_len < MIN_SAMPLES) {
        return false;
    }
    result = select_filtered(hist, hist_len, cfg.near_rank);
    return true;
}

// handler for incoming frames, on the CAN thread. Returns true if the frame
// was for this radar.
bool AP_RangeFinder_Jiyi_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    if (frame.isErrorFrame()) {
        return false;
    }
    const DecodeConfig *cfg = get_config();
    if (cfg == nullptr) {
        return false;
    }
    // The Lua driver matched frame:id() & 0x1FFFFFFF, so a standard or an
    // extended frame carrying 214/220 is accepted, as it was there.
    if ((frame.id & AP_HAL::CANFrame::MaskExtID) != cfg->can_id) {
        // leave it for the other radar's instance
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    float d = 0;
    uint8_t s = 0;
    const Reason r = decode(*cfg, frame, floor_m.get(), snr_min.get(), d, s);

    WITH_SEMAPHORE(_sem);

    if (r == Reason::OK) {
        last_frame_ms = now_ms;
        last_valid_ms = now_ms;
        float m;
        const bool have_m = median_push(*cfg, d, m);
        if (have_m) {
            filtered_m = m;
            filtered_pending = true;
        }
        const float v = have_m ? m : d;
        if (stats.n == 0 || v < stats.min_m) {
            stats.min_m = v;
            stats.s_at_min = s;
        }
        if (stats.n == 0 || v > stats.max_m) {
            stats.max_m = v;
            stats.s_at_max = s;
        }
        stats.n++;
        return true;
    }

    // LIVENESS = a frame that passed header AND checksum, whatever the
    // filters then decide about its contents.
    //   notgt / range / floor / weak -> the radar transmitted a valid frame;
    //     we simply chose not to use the reading. Still alive.
    //   dlc / hdr / chk -> not a valid frame from this sensor.
    // Counting only "notgt" here (as early Lua versions did) made a radar
    // whose returns were merely weak look DEAD after 500 ms, which withheld
    // the synthetic clear, let RNGFND2 age to NoData and dropped PRX1 in
    // flight. Observed airborne over a field at 5.8 m.
    switch (r) {
    case Reason::DLC:
    case Reason::HDR:
    case Reason::CHK:
        stats.bad++;
        break;
    case Reason::NOTGT:
        last_frame_ms = now_ms;
        stats.notgt++;
        break;
    default:
        last_frame_ms = now_ms;
        stats.filtered++;
        break;
    }
    // only a sustained absence of usable readings clears the history, never a
    // single reject (see note 3 at the top of this file)
    if (now_ms - last_valid_ms > STALE_MS) {
        hist_len = 0;
    }
    stats.why = r;
    return true;
}

void AP_RangeFinder_Jiyi_CAN::update(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    const DecodeConfig *cfg = get_config();

    if (cfg != active_cfg) {
        // role chosen (at boot) or changed through RECV_ID: apply that role's
        // defaults where the user has not set a value, and start from scratch
        active_cfg = cfg;
        if (cfg != nullptr) {
            snr_min.set_default(cfg->min_strength);
            enable_ch.set_default(cfg->enable_ch);
            idle_stand_down.set_default(cfg->idle_stand_down);
            warn_estop(*cfg);
        }
        {
            WITH_SEMAPHORE(_sem);
            hist_len = 0;
            last_frame_ms = 0;
            last_valid_ms = 0;
            filtered_pending = false;
            stats = {};
        }
        have_report = false;
        last_publish_ms = 0;
        switch_state = -1;
        set_status(RangeFinder::Status::NotConnected);
    }
    if (cfg == nullptr) {
        // not configured as a Jiyi radar; prearm says why
        return;
    }

    // ahead of the liveness gate on purpose: the switch position is worth
    // reporting even if the bus never comes up
    announce_switch(*cfg);

    Window w;
    load_window(*cfg, w);
    const bool down = stood_down();

    uint32_t frame_ms;
    uint32_t valid_ms;
    bool got_filtered = false;
    float m = 0;
    {
        WITH_SEMAPHORE(_sem);
        frame_ms = last_frame_ms;
        valid_ms = last_valid_ms;
        if (filtered_pending) {
            got_filtered = true;
            m = filtered_m;
            filtered_pending = false;
        }
    }

    // a fresh filter output is what the Lua driver sent from its frame loop;
    // while stood down it is withheld
    if (got_filtered && !down) {
        report_m = clamp_to_window(*cfg, w, m);
        have_report = true;
    }

    // Refresh on every update so ArduPilot never sees a gap longer than one
    // update period. Liveness gates the SYNTHETIC reading only and is never
    // announced: a radar that goes silent simply stops being fed, falls to
    // NoData and blocks arming.
    const bool alive = frame_ms != 0 && (now_ms - frame_ms) <= SENSOR_ALIVE_MS;
    if (alive) {
        if (down || (now_ms - valid_ms) > STALE_MS) {
            // Forward: in-range clear so PRX1 stays Good.
            // Terrain: out-of-range-high -- alive but !Good, never fake clearance.
            report_m = clamp_to_window(*cfg, w, cfg->synth_clear ? w.clear_m : w.no_return_m);
            have_report = true;
            synth_count++;
        }
        if (have_report) {
            publish(report_m, now_ms);
        }
    }
    if (last_publish_ms != 0 && (now_ms - last_publish_ms) > PUBLISH_TIMEOUT_MS) {
        set_status(RangeFinder::Status::NoData);
    }

    if (now_ms - last_stats_ms >= 1000) {
        last_stats_ms = now_ms;
        report_stats(*cfg, alive);
    }
}

void AP_RangeFinder_Jiyi_CAN::publish(float d_m, uint32_t now_ms)
{
    state.distance_m = d_m;
    state.last_reading_ms = now_ms;
    state.signal_quality_pct = RangeFinder::SIGNAL_QUALITY_UNKNOWN;
    update_status();
    last_publish_ms = now_ms;
}

/*
  Usable reporting window for this radar. RNGFNDn_MIN_CM/MAX_CM are read
  through min/max_distance_cm(), so the later rename to RNGFNDn_MIN/MAX (m)
  is handled at compile time rather than by probing names as the Lua did.
  PRX1_MIN/MAX gate proximity independently of the rangefinder window: a
  clear valid for RNGFND2 but outside PRX1's window is discarded and PRX1
  goes NoData, so the obstacle radar intersects both.
 */
void AP_RangeFinder_Jiyi_CAN::load_window(const DecodeConfig &cfg, Window &w) const
{
    w.fault = WindowFault::NONE;
    const float p_max = max_distance_cm() * 0.01f;
    if (p_max <= 0) {
        w.fault = WindowFault::RANGE_UNSET;
        w.min_m = 0.5f;
        w.max_m = 20.0f;
    } else {
        w.min_m = min_distance_cm() * 0.01f;
        w.max_m = p_max;
    }

    w.win_lo = w.min_m;
    w.win_hi = w.max_m;
    if (cfg.synth_clear) {
        float prx_min, prx_max;
        if (get_prx_param(PRX1_MIN, prx_min) && prx_min > 0 && prx_min > w.win_lo) {
            w.win_lo = prx_min;
        }
        if (get_prx_param(PRX1_MAX, prx_max) && prx_max > 0 && prx_max < w.win_hi) {
            w.win_hi = prx_max;
        }
        if (w.win_lo >= w.win_hi) {
            w.fault = WindowFault::PRX_CONFLICT;
            w.win_lo = w.min_m;
            w.win_hi = w.max_m;
        }
    }

    const float margin = MAX(0.3f, w.win_hi * 0.03f);
    w.clear_m = w.win_hi - margin;
    if (w.clear_m <= w.win_lo) {
        w.clear_m = (w.win_lo + w.win_hi) * 0.5f;
    }
    w.no_return_m = w.max_m + 5.0f;
    w.clamp_lo = w.win_lo + 0.05f;
    w.clamp_hi = w.win_hi - 0.05f;
    if (w.clamp_lo >= w.clamp_hi) {
        w.clamp_lo = w.win_lo;
        w.clamp_hi = w.win_hi;
    }
}

float AP_RangeFinder_Jiyi_CAN::clamp_to_window(const DecodeConfig &cfg, const Window &w, float v)
{
    if (!cfg.clamp_to_window) {
        return v;
    }
    // Keep real readings inside the usable window so they land on Good. A
    // target closer than the minimum is reported AT the minimum rather than
    // dropped: with AVOID_MARGIN at 7-10 m anything that close is already
    // deep inside the margin, so avoidance is unchanged, but PRX1 stays
    // healthy instead of going No Data.
    return constrain_float(v, w.clamp_lo, w.clamp_hi);
}

// get the RCn_OPTION of the enable channel; false if there is no enable channel
bool AP_RangeFinder_Jiyi_CAN::get_enable_option(int16_t &option) const
{
    const int8_t ch = enable_ch.get();
    if (ch <= 0) {
        return false;
    }
    const RC_Channel *c = rc().channel(ch - 1);
    if (c == nullptr) {
        return false;
    }
    option = c->option.get();
    return true;
}

bool AP_RangeFinder_Jiyi_CAN::ch_high() const
{
    const int8_t ch = enable_ch.get();
    if (ch <= 0) {
        // no switch configured
        return true;
    }
    const RC_Channel *c = rc().channel(ch - 1);
    const int16_t pwm = (c == nullptr) ? 0 : c->get_radio_in();
    if (pwm <= 0) {
        // channel absent: fail SAFE-ON
        return true;
    }
    return pwm >= RC_ENABLE_US;
}

bool AP_RangeFinder_Jiyi_CAN::stood_down() const
{
    if (ch_high()) {
        return false;
    }
    return idle_stand_down.get() != 0;
}

// Announce the enable switch at startup and on every change:
//   "JIYI: obstacle ON"   "JIYI: terrain OFF"
// The firmware already announces RC8 itself because RC8_OPTION=40, but RC9
// has no aux function so only this driver reports the terrain radar.
void AP_RangeFinder_Jiyi_CAN::announce_switch(const DecodeConfig &cfg)
{
    const int8_t ch = enable_ch.get();
    const RC_Channel *c = (ch > 0) ? rc().channel(ch - 1) : nullptr;
    if (c == nullptr) {
        return;
    }
    const int16_t pwm = c->get_radio_in();
    if (pwm <= 0) {
        // wait for a real PWM, so the switch does not appear to flip once at
        // boot while the RC link is still coming up
        return;
    }
    const int8_t on = (pwm >= RC_ENABLE_US) ? 1 : 0;
    if (on != switch_state) {
        switch_state = on;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "JIYI: %s %s", cfg.label, on ? "ON" : "OFF");
    }
}

static bool is_estop_option(int16_t option)
{
    return option == int16_t(RC_Channel::AUX_FUNC::MOTOR_ESTOP) ||
           option == int16_t(RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP);
}

// Each radar's enable channel is a switch the PILOT flips. If it also drives
// motor emergency stop, flipping it to change a radar cuts the motors. This
// is on top of the prearm failure because prearm checks can be disabled.
void AP_RangeFinder_Jiyi_CAN::warn_estop(const DecodeConfig &cfg) const
{
    int16_t option;
    if (!get_enable_option(option) || !is_estop_option(option)) {
        return;
    }
    for (uint8_t i = 0; i < 3; i++) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "JIYI DANGER: RC%d=%d MOTOR E-STOP! set %d",
                      int(enable_ch.get()), int(option), int(cfg.expect_option));
    }
}

static bool can_protocol_configured()
{
    for (uint8_t i = 0; i < HAL_MAX_CAN_PROTOCOL_DRIVERS; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::RadarCAN) {
            return true;
        }
    }
    return false;
}

bool AP_RangeFinder_Jiyi_CAN::get_prx_param(PrxParam idx, float &value)
{
    CachedParam &cp = prx_params[idx];
    if (!cp.looked_up) {
        cp.looked_up = true;
        cp.ptr = AP_Param::find(cp.name, &cp.type);
    }
    if (cp.ptr == nullptr) {
        return false;
    }
    value = cp.ptr->cast_to_float(cp.type);
    return true;
}

// the proximity settings the forward radar depends on; each of these has
// bitten this installation at least once
bool AP_RangeFinder_Jiyi_CAN::check_proximity_config(char *failure_msg, uint8_t failure_msg_len)
{
    float v;
    if (!get_prx_param(PRX1_TYPE, v) || int(v) != PRX_TYPE_RANGEFINDER) {
        hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: PRX1_TYPE must be %u", unsigned(PRX_TYPE_RANGEFINDER));
        return false;
    }
    // gates proximity while on the ground, i.e. exactly when prearm runs.
    // PRX_ALT_MIN DEFAULTS TO 1.0.
    if (get_prx_param(PRX_ALT_MIN, v) && v > 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: PRX_ALT_MIN=%.1f, set 0", (double)v);
        return false;
    }
    if (get_prx_param(PRX_IGN_GND, v) && !is_zero(v)) {
        hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: PRX_IGN_GND must be 0");
        return false;
    }
    for (uint8_t i = 0; i < 4; i++) {
        if (get_prx_param(PrxParam(PRX1_IGN_WID1 + i), v) && !is_zero(v)) {
            hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: PRX1_IGN_WID%u must be 0", unsigned(i + 1));
            return false;
        }
    }
    return true;
}

// Configuration checks. The Lua driver printed these once at boot; here they
// block arming until fixed.
bool AP_RangeFinder_Jiyi_CAN::prearm_healthy(char *failure_msg, uint8_t failure_msg_len) const
{
    const unsigned n = instance_num() + 1;
    const DecodeConfig *cfg = get_config();
    if (cfg == nullptr) {
        hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: RNGFND%X_RECV_ID must be %u or %u",
                           n, unsigned(ID_TERRAIN), unsigned(ID_OBSTACLE));
        return false;
    }

    // two instances decoding the same radar would leave the other one unused
    const RangeFinder *rf = AP::rangefinder();
    for (uint8_t i = 0; rf != nullptr && i < rf->num_sensors(); i++) {
        const AP_RangeFinder_Backend *b = rf->get_backend(i);
        if (b == nullptr || b == this || b->allocated_type() != RangeFinder::Type::JIYI_CAN) {
            continue;
        }
        if (static_cast<const AP_RangeFinder_Jiyi_CAN *>(b)->receive_id.get() == receive_id.get()) {
            hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: RNGFND%X and %X both RECV_ID %d",
                               n, unsigned(i + 1), int(receive_id.get()));
            return false;
        }
    }

    // the CAN configuration that once took this installation out entirely
    if (!can_protocol_configured()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: no CAN port with CAN_Dx_PROTOCOL=14");
        return false;
    }

    const int8_t ch = enable_ch.get();
    if (ch > 0) {
        int16_t option;
        if (!get_enable_option(option)) {
            hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: RNGFND%X_JY_ENCH=%d invalid", n, int(ch));
            return false;
        }
        if (is_estop_option(option)) {
            hal.util->snprintf(failure_msg, failure_msg_len, "JIYI DANGER: RC%d_OPTION=%d is MOTOR E-STOP", int(ch), int(option));
            return false;
        }
        if (option != cfg->expect_option) {
            hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: %s RC%d_OPTION=%d, want %d",
                               cfg->label, int(ch), int(option), int(cfg->expect_option));
            return false;
        }
    }

    // the forward radar must sit in a horizontal sector or PRX1 ignores it
    if (cfg->synth_clear) {
        const int8_t orient = params.orientation.get();
        if (orient < ROTATION_NONE || orient > ROTATION_YAW_315) {
            hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: RNGFND%X_ORIENT must be 0-7", n);
            return false;
        }
    }

    Window w;
    load_window(*cfg, w);
    switch (w.fault) {
    case WindowFault::NONE:
        break;
    case WindowFault::RANGE_UNSET:
        hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: RNGFND%X_MAX_CM must be >0", n);
        return false;
    case WindowFault::PRX_CONFLICT:
        hal.util->snprintf(failure_msg, failure_msg_len, "JIYI: PRX1_MIN/MAX conflicts with RNGFND%X", n);
        return false;
    }

    if (cfg->synth_clear && !check_proximity_config(failure_msg, failure_msg_len)) {
        return false;
    }

    return true;
}

uint8_t AP_RangeFinder_Jiyi_CAN::instance_num() const
{
    if (instance_idx < 0) {
        const RangeFinder *rf = AP::rangefinder();
        for (uint8_t i = 0; rf != nullptr && i < RANGEFINDER_MAX_INSTANCES; i++) {
            if (rf->get_backend(i) == this) {
                instance_idx = i;
                break;
            }
        }
    }
    return (instance_idx < 0) ? 0 : instance_idx;
}

const char *AP_RangeFinder_Jiyi_CAN::reason_str(Reason r)
{
    switch (r) {
    case Reason::OK:    return "-";
    case Reason::DLC:   return "dlc";
    case Reason::HDR:   return "hdr";
    case Reason::CHK:   return "chk";
    case Reason::NOTGT: return "notgt";
    case Reason::RANGE: return "range";
    case Reason::FLOOR: return "floor";
    case Reason::LOW_STRENGTH: return "weak";
    }
    return "?";
}

// once a second: always to the dataflash log, and to the GCS if JY_DIAG is set
void AP_RangeFinder_Jiyi_CAN::report_stats(const DecodeConfig &cfg, bool alive)
{
    Stats st;
    {
        WITH_SEMAPHORE(_sem);
        st = stats;
        stats = {};
    }
    const uint16_t synth = synth_count;
    synth_count = 0;

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: JIYI
    // @Description: Jiyi CAN radar decode statistics, once per second per radar
    // @Field: TimeUS: Time since system startup
    // @Field: I: rangefinder instance (0-based)
    // @Field: Val: frames with a usable distance
    // @Field: NoT: decodable frames reporting no target
    // @Field: Flt: decodable frames rejected by the range, floor or strength filters
    // @Field: Bad: frames rejected for length, header or checksum
    // @Field: Syn: synthetic reports published
    // @Field: Mn: smallest reading
    // @Field: Mx: largest reading
    // @Field: Out: value currently being reported, -1 if none
    // @Field: Al: 1 if the radar has sent a decodable frame within SENSOR_ALIVE_MS
    AP::logger().Write("JIYI", "TimeUS,I,Val,NoT,Flt,Bad,Syn,Mn,Mx,Out,Al", "QBHHHHHfffB",
                       AP_HAL::micros64(),
                       instance_num(),
                       st.n,
                       st.notgt,
                       st.filtered,
                       st.bad,
                       synth,
                       st.n > 0 ? st.min_m : -1.0f,
                       st.n > 0 ? st.max_m : -1.0f,
                       have_report ? report_m : -1.0f,
                       uint8_t(alive));
#endif

    if (diag.get() == 0) {
        return;
    }
    const int ch = enable_ch.get();
    const char *sw = ch_high() ? "HI" : "LO";
    const unsigned rej = unsigned(st.notgt) + st.filtered + st.bad;
    if (st.n > 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s RC%d:%s n%u %.2f-%.2fm sN%u sF%u r%u y%u",
                      cfg.name, ch, sw, unsigned(st.n), (double)st.min_m, (double)st.max_m,
                      unsigned(st.s_at_min), unsigned(st.s_at_max), rej, unsigned(synth));
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s RC%d:%s none r%u %s y%u %s",
                      cfg.name, ch, sw, rej, reason_str(st.why), unsigned(synth),
                      alive ? "alive" : "DEAD");
    }
}

#endif  // AP_RANGEFINDER_JIYI_CAN_ENABLED
