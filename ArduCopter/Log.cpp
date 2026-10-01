#include "Copter.h"

#if HAL_LOGGING_ENABLED

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    throttle_in;
    float    angle_boost;
    float    throttle_out;
    float    throttle_hover;
    float    desired_alt;
    float    inav_alt;
    int32_t  baro_alt;
    float    desired_rangefinder_alt;
    float    rangefinder_alt;
    float    terr_alt;
    int16_t  target_climb_rate;
    int16_t  climb_rate;
};

// Write a control tuning packet
void Copter::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE
    if (!terrain.height_above_terrain(terr_alt, true)) {
        terr_alt = logger.quiet_nan();
    }
#endif
    float des_alt_m = 0.0f;
    int16_t target_climb_rate_cms = 0;
    if (!flightmode->has_manual_throttle()) {
        des_alt_m = pos_control->get_pos_target_z_cm() * 0.01f;
        target_climb_rate_cms = pos_control->get_vel_target_z_cms();
    }

    float desired_rangefinder_alt;
#if AP_RANGEFINDER_ENABLED
    if (!surface_tracking.get_target_dist_for_logging(desired_rangefinder_alt)) {
        desired_rangefinder_alt = AP::logger().quiet_nan();
    }
#else
    // get surface tracking alts
    desired_rangefinder_alt = AP::logger().quiet_nan();
#endif

    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control->get_throttle_in(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        throttle_hover      : motors->get_throttle_hover(),
        desired_alt         : des_alt_m,
        inav_alt            : inertial_nav.get_position_z_up_cm() * 0.01f,
        baro_alt            : baro_alt,
        desired_rangefinder_alt : desired_rangefinder_alt,
#if AP_RANGEFINDER_ENABLED
        rangefinder_alt     : surface_tracking.get_dist_for_logging(),
#else
        rangefinder_alt     : AP::logger().quiet_nanf(),
#endif
        terr_alt            : terr_alt,
        target_climb_rate   : target_climb_rate_cms,
        climb_rate          : int16_t(inertial_nav.get_velocity_z_up_cms()) // float -> int16_t
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void Copter::Log_Write_Attitude()
{
    attitude_control->Write_ANG();
    attitude_control->Write_Rate(*pos_control);
}

// Write PIDS packets
void Copter::Log_Write_PIDS()
{
   if (should_log(MASK_LOG_PID)) {
        logger.Write_PID(LOG_PIDR_MSG, attitude_control->get_rate_roll_pid().get_pid_info());
        logger.Write_PID(LOG_PIDP_MSG, attitude_control->get_rate_pitch_pid().get_pid_info());
        logger.Write_PID(LOG_PIDY_MSG, attitude_control->get_rate_yaw_pid().get_pid_info());
        logger.Write_PID(LOG_PIDA_MSG, pos_control->get_accel_z_pid().get_pid_info() );
        if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS())) {
            logger.Write_PID(LOG_PIDN_MSG, pos_control->get_vel_xy_pid().get_pid_info_x());
            logger.Write_PID(LOG_PIDE_MSG, pos_control->get_vel_xy_pid().get_pid_info_y());
        }
    }
}

// Write an EKF and POS packet
void Copter::Log_Write_EKF_POS()
{
    AP::ahrs().Log_Write();
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
UNUSED_FUNCTION 
void Copter::Log_Write_Data(LogDataID id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
void Copter::Log_Write_Data(LogDataID id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            time_us  : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
void Copter::Log_Write_Data(LogDataID id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    float data_value;
};

// Write a float data packet
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, float value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_ParameterTuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  parameter;     // parameter we are tuning, e.g. 39 is CH6_CIRCLE_RATE
    float    tuning_value;  // normalized value used inside tuning() function
    float    tuning_min;    // tuning minimum value
    float    tuning_max;    // tuning maximum value
};

void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
        time_us        : AP_HAL::micros64(),
        parameter      : param,
        tuning_value   : tuning_val,
        tuning_min     : tune_min,
        tuning_max     : tune_max
    };

    logger.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}

void Copter::Log_Video_Stabilisation()
{
    if (!should_log(MASK_LOG_VIDEO_STABILISATION)) {
        return;
    }
    ahrs.write_video_stabilisation();
}

struct PACKED log_SysIdD {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    waveform_time;
    float    waveform_sample;
    float    waveform_freq;
    float    angle_x;
    float    angle_y;
    float    angle_z;
    float    accel_x;
    float    accel_y;
    float    accel_z;
};

// Write an rate packet
void Copter::Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z)
{
#if MODE_SYSTEMID_ENABLED
    struct log_SysIdD pkt_sidd = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDD_MSG),
        time_us         : AP_HAL::micros64(),
        waveform_time   : waveform_time,
        waveform_sample : waveform_sample,
        waveform_freq   : waveform_freq,
        angle_x         : angle_x,
        angle_y         : angle_y,
        angle_z         : angle_z,
        accel_x         : accel_x,
        accel_y         : accel_y,
        accel_z         : accel_z
    };
    logger.WriteBlock(&pkt_sidd, sizeof(pkt_sidd));
#endif
}

struct PACKED log_SysIdS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  systemID_axis;
    float    waveform_magnitude;
    float    frequency_start;
    float    frequency_stop;
    float    time_fade_in;
    float    time_const_freq;
    float    time_record;
    float    time_fade_out;
};

// Write an rate packet
void Copter::Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out)
{
#if MODE_SYSTEMID_ENABLED
    struct log_SysIdS pkt_sids = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDS_MSG),
        time_us             : AP_HAL::micros64(),
        systemID_axis       : systemID_axis,
        waveform_magnitude  : waveform_magnitude,
        frequency_start     : frequency_start,
        frequency_stop      : frequency_stop,
        time_fade_in        : time_fade_in,
        time_const_freq     : time_const_freq,
        time_record         : time_record,
        time_fade_out       : time_fade_out
    };
    logger.WriteBlock(&pkt_sids, sizeof(pkt_sids));
#endif
}

// guided position target logging
struct PACKED log_Guided_Position_Target {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    uint8_t terrain;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
    float accel_target_x;
    float accel_target_y;
    float accel_target_z;
};

// guided attitude target logging
struct PACKED log_Guided_Attitude_Target {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float thrust;
    float climb_rate;
};

// Write a Guided mode position target
// pos_target is lat, lon, alt OR offset from ekf origin in cm
// terrain should be 0 if pos_target.z is alt-above-ekf-origin, 1 if alt-above-terrain
// vel_target is cm/s
void Copter::Log_Write_Guided_Position_Target(ModeGuided::SubMode target_type, const Vector3f& pos_target, bool terrain_alt, const Vector3f& vel_target, const Vector3f& accel_target)
{
    const log_Guided_Position_Target pkt {
        LOG_PACKET_HEADER_INIT(LOG_GUIDED_POSITION_TARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : (uint8_t)target_type,
        pos_target_x    : pos_target.x,
        pos_target_y    : pos_target.y,
        pos_target_z    : pos_target.z,
        terrain         : terrain_alt,
        vel_target_x    : vel_target.x,
        vel_target_y    : vel_target.y,
        vel_target_z    : vel_target.z,
        accel_target_x  : accel_target.x,
        accel_target_y  : accel_target.y,
        accel_target_z  : accel_target.z
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write a Guided mode attitude target
// roll, pitch and yaw are in radians
// ang_vel: angular velocity, [roll rate, pitch_rate, yaw_rate] in radians/sec
// thrust is between 0 to 1
// climb_rate is in (m/s)
void Copter::Log_Write_Guided_Attitude_Target(ModeGuided::SubMode target_type, float roll, float pitch, float yaw, const Vector3f &ang_vel, float thrust, float climb_rate)
{
    const log_Guided_Attitude_Target pkt {
        LOG_PACKET_HEADER_INIT(LOG_GUIDED_ATTITUDE_TARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : (uint8_t)target_type,
        roll            : degrees(roll),       // rad to deg
        pitch           : degrees(pitch),      // rad to deg
        yaw             : degrees(yaw),        // rad to deg
        roll_rate       : degrees(ang_vel.x),  // rad/s to deg/s
        pitch_rate      : degrees(ang_vel.y),  // rad/s to deg/s
        yaw_rate        : degrees(ang_vel.z),  // rad/s to deg/s
        thrust          : thrust,
        climb_rate      : climb_rate
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

#if MODE_LOITER_ENABLED
// NOTE: AP_Logger's FMT record carries char format[16] / char labels[64], and FMTU
// carries char units[16] / char multipliers[16].  A log message therefore cannot have
// more than 16 fields, and the label list cannot exceed 64 characters.  Exceeding
// either is truncated silently at write time - it is not caught by any assert on this
// target - and every standard parser then reads the message with the wrong layout.
// Keep this struct at 16 fields and re-count the four strings after any change.
struct PACKED log_LTurn {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t state;
    float state_timer;
    float vx;
    float vy;
    float target_vx;
    float target_vy;
    float accel_x;
    float accel_y;
    float accel_x_raw;
    float accel_y_raw;
    float psc_accel_des;
    float roll_in;
    float pos_error;
    float psc_vel_des;
    uint8_t exit_reason;
};

// Write an L-turn (LTRN) state machine record.  Called every loop while the state
// machine is not INACTIVE, and once more on the loop an exit fires.
// Speeds are m/s and accelerations m/s/s in the latched heading's body frame; the
// roll input is normalised -1 to 1.
// The exit record is written critically so it cannot be dropped under logging load -
// that record is the only place the roll value compared by the exit test appears.
void Copter::Log_Write_LTurn(uint8_t state_in, float state_timer_s,
                             float vx_ms, float vy_ms, float tgt_vx_ms, float tgt_vy_ms,
                             float accel_x_mss, float accel_y_mss,
                             float accel_x_raw_mss, float accel_y_raw_mss,
                             float psc_accel_des_mss, float roll_norm, float pos_err_m,
                             float psc_vel_des_ms, uint8_t exit_reason_in,
                             bool is_critical)
{
    const log_LTurn pkt {
        LOG_PACKET_HEADER_INIT(LOG_LTURN_MSG),
        time_us     : AP_HAL::micros64(),
        state       : state_in,
        state_timer : state_timer_s,
        vx          : vx_ms,
        vy          : vy_ms,
        target_vx   : tgt_vx_ms,
        target_vy   : tgt_vy_ms,
        accel_x     : accel_x_mss,
        accel_y     : accel_y_mss,
        accel_x_raw : accel_x_raw_mss,
        accel_y_raw : accel_y_raw_mss,
        psc_accel_des : psc_accel_des_mss,
        roll_in     : roll_norm,
        pos_error   : pos_err_m,
        psc_vel_des : psc_vel_des_ms,
        exit_reason : exit_reason_in
    };
    if (is_critical) {
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    } else {
        logger.WriteBlock(&pkt, sizeof(pkt));
    }
}
#endif  // MODE_LOITER_ENABLED

#if MODE_AUTO_ENABLED
struct PACKED log_SemiAuto {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float offset;
    float rate;
    float wp_alt;
    uint8_t active;
};

// Write a semi-auto altitude bias record.  Called every loop while in AUTO with
// KFT_SA_ENABLE set.  Used to reconcile actual spray coverage against the
// planned grid during post-flight analysis.  Inputs are in cm and cm/s, logged
// in metres and m/s.
void Copter::Log_Write_SemiAuto(float offset_cm, float rate_cms, float wp_alt_cm, bool active)
{
    const log_SemiAuto pkt {
        LOG_PACKET_HEADER_INIT(LOG_SEMIAUTO_MSG),
        time_us     : AP_HAL::micros64(),
        offset      : offset_cm * 0.01f,
        rate        : rate_cms * 0.01f,
        wp_alt      : wp_alt_cm * 0.01f,
        active      : active
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}
#endif  // MODE_AUTO_ENABLED

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Copter::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    
// @LoggerMessage: PTUN
// @Description: Parameter Tuning information
// @URL: https://ardupilot.org/copter/docs/tuning.html#in-flight-tuning
// @Field: TimeUS: Time since system startup
// @Field: Param: Parameter being tuned
// @Field: TunVal: Normalized value used inside tuning() function
// @Field: TunMin: Tuning minimum limit
// @Field: TunMax: Tuning maximum limit

    { LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
      "PTUN", "QBfff",         "TimeUS,Param,TunVal,TunMin,TunMax", "s----", "F----" },

// @LoggerMessage: CTUN
// @Description: Control Tuning information
// @Field: TimeUS: Time since system startup
// @Field: ThI: throttle input
// @Field: ABst: angle boost
// @Field: ThO: throttle output
// @Field: ThH: calculated hover throttle
// @Field: DAlt: desired altitude
// @Field: Alt: achieved altitude
// @Field: BAlt: barometric altitude
// @Field: DSAlt: desired rangefinder altitude
// @Field: SAlt: achieved rangefinder altitude
// @Field: TAlt: terrain altitude
// @Field: DCRt: desired climb rate
// @Field: CRt: climb rate

// @LoggerMessage: D16
// @Description: Generic 16-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU16
// @Description: Generic 16-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: D32
// @Description: Generic 32-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DFLT
// @Description: Generic float storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU32
// @Description: Generic 32-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qffffffefffhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00B000BB" , true },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "QBh",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "QBH",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "QBi",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "QBI",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "QBf",         "TimeUS,Id,Value", "s--", "F--" },

// @LoggerMessage: SIDD
// @Description: System ID data
// @Field: TimeUS: Time since system startup
// @Field: Time: Time reference for waveform
// @Field: Targ: Current waveform sample
// @Field: F: Instantaneous waveform frequency
// @Field: Gx: Delta angle, X-Axis
// @Field: Gy: Delta angle, Y-Axis
// @Field: Gz: Delta angle, Z-Axis
// @Field: Ax: Delta velocity, X-Axis
// @Field: Ay: Delta velocity, Y-Axis
// @Field: Az: Delta velocity, Z-Axis

    { LOG_SYSIDD_MSG, sizeof(log_SysIdD),
      "SIDD", "Qfffffffff",  "TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az", "ss-zkkkooo", "F---------" , true },

// @LoggerMessage: SIDS
// @Description: System ID settings
// @Field: TimeUS: Time since system startup
// @Field: Ax: The axis which is being excited
// @Field: Mag: Magnitude of the chirp waveform
// @Field: FSt: Frequency at the start of chirp
// @Field: FSp: Frequency at the end of chirp
// @Field: TFin: Time to reach maximum amplitude of chirp
// @Field: TC: Time at constant frequency before chirp starts
// @Field: TR: Time taken to complete chirp waveform
// @Field: TFout: Time to reach zero amplitude after chirp finishes

    { LOG_SYSIDS_MSG, sizeof(log_SysIdS),
      "SIDS", "QBfffffff",  "TimeUS,Ax,Mag,FSt,FSp,TFin,TC,TR,TFout", "s--ssssss", "F--------" , true },

// @LoggerMessage: GUIP
// @Description: Guided mode position target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: pX: Target position, X-Axis
// @Field: pY: Target position, Y-Axis
// @Field: pZ: Target position, Z-Axis
// @Field: Terrain: Target position, Z-Axis is alt above terrain
// @Field: vX: Target velocity, X-Axis
// @Field: vY: Target velocity, Y-Axis
// @Field: vZ: Target velocity, Z-Axis
// @Field: aX: Target acceleration, X-Axis
// @Field: aY: Target acceleration, Y-Axis
// @Field: aZ: Target acceleration, Z-Axis

    { LOG_GUIDED_POSITION_TARGET_MSG, sizeof(log_Guided_Position_Target),
      "GUIP",  "QBfffbffffff",    "TimeUS,Type,pX,pY,pZ,Terrain,vX,vY,vZ,aX,aY,aZ", "s-mmm-nnnooo", "F-BBB-BBBBBB" , true },

// @LoggerMessage: GUIA
// @Description: Guided mode attitude target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: Roll: Target attitude, Roll
// @Field: Pitch: Target attitude, Pitch
// @Field: Yaw: Target attitude, Yaw
// @Field: RollRt: Roll rate
// @Field: PitchRt: Pitch rate
// @Field: YawRt: Yaw rate
// @Field: Thrust: Thrust 
// @Field: ClimbRt: Climb rate

    { LOG_GUIDED_ATTITUDE_TARGET_MSG, sizeof(log_Guided_Attitude_Target),
      "GUIA",  "QBffffffff",    "TimeUS,Type,Roll,Pitch,Yaw,RollRt,PitchRt,YawRt,Thrust,ClimbRt", "s-dddkkk-n", "F-000000-0" , true },

#if MODE_LOITER_ENABLED
// @LoggerMessage: LTRN
// @Description: Loiter L-Turn state machine. Logged every loop while an L-Turn is active, plus one critical record on the loop the manoeuvre exits.
// @Field: TimeUS: Time since system startup
// @Field: St: L-Turn state (0:Inactive, 1:Braking, 2:Lateral, 3:ExitStopX, 4:ExitStopY, 5:ExitBlend)
// @Field: STmr: Time spent in the current state (restarts at the exit stop's X to Y stage change)
// @Field: VX: Current body-X speed
// @Field: VY: Current body-Y speed
// @Field: TVX: Commanded body-X speed (zero except during the exit stop)
// @Field: TVY: Commanded body-Y speed
// @Field: AX: Body-X acceleration commanded after the vector clamp and jerk slew
// @Field: AY: Body-Y acceleration commanded after the vector clamp and jerk slew
// @Field: AXR: Body-X acceleration demand before the vector clamp
// @Field: AYR: Body-Y acceleration demand before the vector clamp
// @Field: DA: Magnitude of PosControl's desired (feed forward) horizontal acceleration - the value Loiter overwrites at handover
// @Field: RIn: Normalised pilot roll input, the exact value the exit test compared
// @Field: PErr: Horizontal position controller error magnitude
// @Field: DV: Magnitude of PosControl's desired horizontal velocity - the value Loiter inherits at handover. Total measured speed is |VX,VY|
// @Field: ER: Exit reason (0:None, 1:StickCentred, 2:RCLoss, 3:EKF, 4:Battery, 5:Landed, 6:NotFlying, 7:ModeChange, 8:Stopped, 9:ExitTimeout, 10:ExitDistance, 11:HandoverTimeout, 12:StickBlend)

    { LOG_LTURN_MSG, sizeof(log_LTurn),
      "LTRN",  "QBfffffffffffffB",  "TimeUS,St,STmr,VX,VY,TVX,TVY,AX,AY,AXR,AYR,DA,RIn,PErr,DV,ER", "s-snnnnooooo-mn-", "F-0000000000000-" , true },
#endif

#if MODE_AUTO_ENABLED
// @LoggerMessage: SAOF
// @Description: Semi-auto altitude bias. Logged every loop while in AUTO with KFT_SA_ENABLE set.
// @Field: TimeUS: Time since system startup
// @Field: Ofs: Altitude bias currently applied on top of the planned mission altitude
// @Field: Rate: Rate of change of the bias commanded by the pilot's throttle stick
// @Field: WPAlt: Planned altitude of the current waypoint above home
// @Field: Act: 1 if the pilot is currently permitted to move the bias

    { LOG_SEMIAUTO_MSG, sizeof(log_SemiAuto),
      "SAOF",  "QfffB",  "TimeUS,Ofs,Rate,WPAlt,Act", "smnm-", "F0000" , true },
#endif
};

uint8_t Copter::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

void Copter::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    char frame_and_type_string[30];
    copter.motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    logger.Write_MessageF("%s", frame_and_type_string);
    logger.Write_Mode((uint8_t)flightmode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

#endif // HAL_LOGGING_ENABLED
