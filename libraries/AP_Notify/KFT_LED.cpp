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
// ============================================================================
//  KFT | JIYI K++ style RGB status LED
// ----------------------------------------------------------------------------
//  Native C++ port of KFT-LUA-LED-001 v1.0 (KFT).
//
//  Opt in by including the KFT bit in NTF_LED_TYPES and assigning a servo
//  output to NeoPixel1..4 (SERVOn_FUNCTION 120..123).  The chain length comes
//  from the existing NTF_LED_LEN parameter.  Do not set the NeoPixel or
//  NeoPixelRGB bits at the same time: both backends would drive the same
//  physical chain and fight over it.
//
//  Priority ordered state table, highest first, evaluated every update:
//
//    1. battery < NTF_KFTLED_CRITV * NTF_KFTLED_CELLS  RED fast blink   100ms
//    2. battery < NTF_KFTLED_LOWV  * NTF_KFTLED_CELLS  RED/ORANGE alt   250ms
//    3. armed,    GPS 3D or better                     GREEN heartbeat
//    4. armed,    GPS 2D only                          YELLOW blink     250ms
//    5. armed,    no GPS fix                           ORANGE solid
//    6. disarmed, no valid RC input                    MAGENTA blink    100ms
//    7. disarmed, GPS 3D or better                     GREEN blink      600ms
//    8. disarmed, GPS 2D only                          CYAN blink       250ms
//    9. disarmed, no GPS fix                           RED blink        600ms
//
//  This backend is a pure observer: it never changes arming, failsafe or
//  flight mode state.  Colours are written at full scale and are deliberately
//  not scaled by NTF_LED_BRIGHT, matching the Lua original.
// ============================================================================

#include "KFT_LED.h"

#if AP_NOTIFY_KFT_LED_ENABLED

#include "AP_Notify.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_SerialLED/AP_SerialLED.h>
#include <SRV_Channel/SRV_Channel.h>

#include <AP_BattMonitor/AP_BattMonitor_config.h>
#if AP_BATTERY_ENABLED
#include <AP_BattMonitor/AP_BattMonitor.h>
#endif

#include <RC_Channel/RC_Channel_config.h>
#if AP_RC_CHANNEL_ENABLED
#include <RC_Channel/RC_Channel.h>
#endif

// This limit is from the dshot driver rcout groups limit, matching NeoPixel
#define AP_NOTIFY_KFT_LED_MAX_INSTANCES 4

// blink half-periods (ms), the time for one ON or one OFF phase
#define KFT_LED_T_FAST 100
#define KFT_LED_T_MED  250
#define KFT_LED_T_SLOW 600

// ms per heartbeat step: ON, short off, ON, long off (~1s cycle)
static const uint16_t KFT_LED_BEAT_MS[4] = { 55, 80, 55, 810 };

static const KFT_LED::RGB KFT_LED_OFF     = {   0,   0,   0 };
static const KFT_LED::RGB KFT_LED_RED     = { 255,   0,   0 };
static const KFT_LED::RGB KFT_LED_GREEN   = {   0, 220,   0 };
static const KFT_LED::RGB KFT_LED_YELLOW  = { 255, 160,   0 };
static const KFT_LED::RGB KFT_LED_ORANGE  = { 255,  55,   0 };
static const KFT_LED::RGB KFT_LED_CYAN    = {   0, 180, 220 };
static const KFT_LED::RGB KFT_LED_MAGENTA = { 200,   0, 220 };

// set up the NeoPixel outputs.  This is NeoPixel::init_ports() with the
// NTF_LED_TYPES test removed: the chain is always driven as GRB NeoPixels,
// because the user opts in through the KFT bit rather than the NeoPixel bit.
uint16_t KFT_LED::init_ports()
{
    uint16_t mask = 0;
    for (uint16_t i=0; i<AP_NOTIFY_KFT_LED_MAX_INSTANCES; i++) {
        const SRV_Channel::Aux_servo_function_t fn = (SRV_Channel::Aux_servo_function_t)((uint8_t)SRV_Channel::k_LED_neopixel1 + i);
        if (!SRV_Channels::function_assigned(fn)) {
            continue;
        }
        mask |= SRV_Channels::get_output_channel_mask(fn);
    }

    if (mask == 0) {
        return 0;
    }

    AP_SerialLED *led = AP_SerialLED::get_singleton();
    if (led == nullptr) {
        return 0;
    }

    for (uint16_t chan=0; chan<16; chan++) {
        if ((1U<<chan) & mask) {
            led->set_num_neopixel(chan+1, pNotify->get_led_len());
        }
    }

    return mask;
}

void KFT_LED::led_set(const RGB &colour)
{
    hw_set_rgb(colour.r, colour.g, colour.b);
}

void KFT_LED::led_blink(const RGB &colour, uint16_t period_ms)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _blink_timer_ms >= period_ms) {
        _blink_timer_ms = now_ms;
        _blink_phase = !_blink_phase;
    }
    led_set(_blink_phase ? colour : KFT_LED_OFF);
}

void KFT_LED::led_alt(const RGB &colour_a, const RGB &colour_b, uint16_t period_ms)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _blink_timer_ms >= period_ms) {
        _blink_timer_ms = now_ms;
        _blink_phase = !_blink_phase;
    }
    led_set(_blink_phase ? colour_a : colour_b);
}

void KFT_LED::led_heartbeat(const RGB &colour)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _beat_timer_ms >= KFT_LED_BEAT_MS[_beat_step]) {
        _beat_timer_ms = now_ms;
        _beat_step = (_beat_step + 1) % 4;
    }
    // steps 0 and 2 are ON, steps 1 and 3 are OFF
    led_set((_beat_step == 0 || _beat_step == 2) ? colour : KFT_LED_OFF);
}

void KFT_LED::update()
{
    const bool armed = AP_Notify::flags.armed;
    // AP_Notify::flags.gps_status carries the AP_GPS::GPS_Status value
    const uint8_t gps_status = AP_Notify::flags.gps_status;

    // ---- priority 1 and 2: battery ------------------------------------
#if AP_BATTERY_ENABLED
    const float volts = AP::battery().voltage();
    const uint8_t cells = pNotify->get_kftled_cells();
    // a voltage of zero means no battery monitor is configured, in which case
    // the battery states are skipped entirely
    if (is_positive(volts) && cells > 0) {
        if (volts < pNotify->get_kftled_crit_volt() * cells) {
            led_blink(KFT_LED_RED, KFT_LED_T_FAST);
            return;
        }
        if (volts < pNotify->get_kftled_low_volt() * cells) {
            led_alt(KFT_LED_RED, KFT_LED_ORANGE, KFT_LED_T_MED);
            return;
        }
    }
#endif  // AP_BATTERY_ENABLED

    // ---- priority 3 to 5: armed ---------------------------------------
    if (armed) {
        if (gps_status >= AP_GPS::GPS_OK_FIX_3D) {
            led_heartbeat(KFT_LED_GREEN);
        } else if (gps_status == AP_GPS::GPS_OK_FIX_2D) {
            led_blink(KFT_LED_YELLOW, KFT_LED_T_MED);
        } else {
            led_set(KFT_LED_ORANGE);
        }
        return;
    }

    // ---- priority 6 to 9: disarmed ------------------------------------
#if AP_RC_CHANNEL_ENABLED
    const RC_Channels *rcin = RC_Channels::get_singleton();
    if (rcin == nullptr || !rcin->has_valid_input()) {
        led_blink(KFT_LED_MAGENTA, KFT_LED_T_FAST);
        return;
    }
#endif

    if (gps_status >= AP_GPS::GPS_OK_FIX_3D) {
        led_blink(KFT_LED_GREEN, KFT_LED_T_SLOW);
    } else if (gps_status == AP_GPS::GPS_OK_FIX_2D) {
        led_blink(KFT_LED_CYAN, KFT_LED_T_MED);
    } else {
        led_blink(KFT_LED_RED, KFT_LED_T_SLOW);
    }
}

#endif  // AP_NOTIFY_KFT_LED_ENABLED
