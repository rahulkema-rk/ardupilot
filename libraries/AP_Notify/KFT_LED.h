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
  KFT | JIYI K++ style RGB status LED

  Native C++ port of KFT-LUA-LED-001 v1.0.  Drives a WS2812B NeoPixel chain
  through the standard serial LED output and paints a fixed, priority ordered
  status pattern.  See KFT_LED.cpp for the state table.
 */
#pragma once

#include "AP_Notify_config.h"

#if AP_NOTIFY_KFT_LED_ENABLED

#include "NeoPixel.h"

class KFT_LED: public NeoPixel {
public:

    struct RGB {
        uint8_t r, g, b;
    };

    // always drive the chain as GRB NeoPixels, regardless of which other
    // NTF_LED_TYPES bits happen to be set
    uint16_t init_ports() override;

    // update - called at 50Hz by AP_Notify
    void update() override;

private:

    // symmetric blink between colour and off
    void led_blink(const RGB &colour, uint16_t period_ms);

    // alternating blink between two colours
    void led_alt(const RGB &colour_a, const RGB &colour_b, uint16_t period_ms);

    // JIYI style double-pulse heartbeat
    void led_heartbeat(const RGB &colour);

    // push a colour to the whole chain
    void led_set(const RGB &colour);

    // shared blink state, matching the single blink timer of the Lua original
    uint32_t _blink_timer_ms{0};
    bool _blink_phase{false};

    // heartbeat sub-state, 0..3: ON / short off / ON / long off
    uint32_t _beat_timer_ms{0};
    uint8_t _beat_step{0};
};

#endif  // AP_NOTIFY_KFT_LED_ENABLED
