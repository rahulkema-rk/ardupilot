#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_JIYI_CAN_ENABLED

#include "AP_RangeFinder_Backend_CAN.h"

/*
  Jiyi 24GHz FMCW CAN radar, used as a pair (terrain/down + obstacle/forward)
  on a Jiyi CAN Hub-12. Native port of the field-validated jiyi_radar.lua.

  One backend instance per radar; RNGFNDn_RECV_ID selects which one:
    214  terrain  (downward, surface tracking)
    220  obstacle (forward, feeds PRX1 via PRX1_TYPE=4)
  Both share one bus (CAN_Dx_PROTOCOL=14, RadarCAN) and each instance only
  consumes frames carrying its own ID.
 */
class AP_RangeFinder_Jiyi_CAN : public AP_RangeFinder_Backend_CAN {
public:
    AP_RangeFinder_Jiyi_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    static const struct AP_Param::GroupInfo var_info[];

    // CAN IDs, and the valid values of RNGFNDn_RECV_ID
    static constexpr uint16_t ID_TERRAIN = 214;
    static constexpr uint16_t ID_OBSTACLE = 220;

    // configuration checks, called from RangeFinder::prearm_healthy()
    bool prearm_healthy(char *failure_msg, uint8_t failure_msg_len) const;

    // per-ID frame format and policy; direct port of DECODE[] in jiyi_radar.lua
    struct DecodeConfig {
        uint16_t can_id;
        const char *name;           // "T"/"O", tag used in diagnostic text
        const char *label;          // "terrain"/"obstacle", used in switch announcements
        uint8_t header[4];          // bytes 0..3, constant per sensor
        uint8_t chk_lo, chk_hi;     // byte 7 == sum(bytes chk_lo..chk_hi) & 0xFF
        uint8_t dist_hi, dist_lo;   // distance, uint16 big-endian
        float scale_to_m;
        float sane_min_m, sane_max_m;
        uint8_t near_rank;          // 0: plain median, N: Nth smallest of the window
        bool synth_clear;           // true: synth_mode "clear", false: "out_of_range"
        bool clamp_to_window;
        int8_t enable_ch;           // role default for JY_ENCH
        bool idle_stand_down;       // role default for JY_IDLE
        uint8_t min_strength;       // role default for SNR_MIN
        int16_t expect_option;      // RCn_OPTION expected on the enable channel
    };
    static const DecodeConfig DECODE[2];

    // outcome of decoding one frame. DLC/HDR/CHK: not a valid frame from this
    // sensor. Every other outcome proves the sensor is alive.
    // (LOW_STRENGTH is Lua's "weak"; WEAK is an ArduPilot macro)
    enum class Reason : uint8_t { OK, DLC, HDR, CHK, NOTGT, RANGE, FLOOR, LOW_STRENGTH };

    // decode one frame; on OK fills dist_m. strength is filled once the
    // checksum has passed. Static so it can be unit tested.
    static Reason decode(const DecodeConfig &cfg, const AP_HAL::CANFrame &frame,
                         float floor_m, int32_t min_strength,
                         float &dist_m, uint8_t &strength);

    // pick the reported value from an unsorted history window (len >= 1)
    static float select_filtered(const float *values, uint8_t len, uint8_t near_rank);

    static constexpr uint8_t MEDIAN_N_MAX = 15;

protected:
    void update() override;
    bool handle_frame(AP_HAL::CANFrame &frame) override;

private:
    enum class WindowFault : uint8_t { NONE, RANGE_UNSET, PRX_CONFLICT };

    // usable reporting window and synthetic values, see load_window()
    struct Window {
        float min_m, max_m;         // RNGFNDn_MIN_CM/MAX_CM
        float win_lo, win_hi;       // intersected with PRX1_MIN/MAX for the obstacle radar
        float clear_m;              // synthetic in-range "clear" (obstacle only)
        float no_return_m;          // synthetic out-of-range-high (terrain)
        float clamp_lo, clamp_hi;
        WindowFault fault;
    };

    // proximity parameters looked up by name, as the Lua driver did
    enum PrxParam : uint8_t {
        PRX1_TYPE, PRX1_MIN, PRX1_MAX, PRX_ALT_MIN, PRX_IGN_GND,
        PRX1_IGN_WID1, PRX1_IGN_WID2, PRX1_IGN_WID3, PRX1_IGN_WID4,
        PRX_PARAM_COUNT
    };
    struct CachedParam {
        const char *name;
        AP_Param *ptr;
        ap_var_type type;
        bool looked_up;
    };
    static CachedParam prx_params[PRX_PARAM_COUNT];
    static bool get_prx_param(PrxParam idx, float &value);
    static bool check_proximity_config(char *failure_msg, uint8_t failure_msg_len);

    struct Stats {
        uint16_t n;             // frames with a usable distance
        uint16_t notgt;         // decodable frames reporting no target
        uint16_t filtered;      // decodable frames rejected by range/floor/strength
        uint16_t bad;           // dlc/hdr/chk rejects
        float min_m, max_m;
        uint8_t s_at_min, s_at_max;
        Reason why;             // most recent reject
    };

    const DecodeConfig *get_config() const;
    bool median_push(const DecodeConfig &cfg, float d, float &result);
    void load_window(const DecodeConfig &cfg, Window &w) const;
    static float clamp_to_window(const DecodeConfig &cfg, const Window &w, float v);
    void publish(float d_m, uint32_t now_ms);
    bool get_enable_option(int16_t &option) const;
    bool ch_high() const;
    bool stood_down() const;
    void announce_switch(const DecodeConfig &cfg);
    void warn_estop(const DecodeConfig &cfg) const;
    void report_stats(const DecodeConfig &cfg, bool alive);
    uint8_t instance_num() const;
    static const char *reason_str(Reason r);

    // parameters; RECV_ID and SNR_MIN are inherited from AP_RangeFinder_Backend_CAN
    AP_Float floor_m;           // JY_FLR
    AP_Int8 median_n;           // JY_MEDN
    AP_Int8 enable_ch;          // JY_ENCH
    AP_Int8 idle_stand_down;    // JY_IDLE
    AP_Int8 diag;               // JY_DIAG

    // written by handle_frame() on the CAN thread, read by update(); protected by _sem
    float hist[MEDIAN_N_MAX];
    uint8_t hist_len;
    uint32_t last_frame_ms;     // last decodable frame (liveness)
    uint32_t last_valid_ms;     // last frame with a usable distance
    float filtered_m;           // newest filter output not yet taken by update()
    bool filtered_pending;
    Stats stats;

    // main thread only
    const DecodeConfig *active_cfg;
    float report_m;             // Lua last_report: the value that keeps being refreshed
    bool have_report;
    uint32_t last_publish_ms;
    uint32_t last_stats_ms;
    uint16_t synth_count;
    int8_t switch_state = -1;
    mutable int8_t instance_idx = -1;
};

#endif  // AP_RANGEFINDER_JIYI_CAN_ENABLED
