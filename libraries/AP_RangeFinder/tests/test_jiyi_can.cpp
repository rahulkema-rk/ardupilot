#include <AP_gtest.h>

#include <AP_RangeFinder/AP_RangeFinder_Jiyi_CAN.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_RANGEFINDER_JIYI_CAN_ENABLED

using Jiyi = AP_RangeFinder_Jiyi_CAN;
using Reason = AP_RangeFinder_Jiyi_CAN::Reason;

static const Jiyi::DecodeConfig &terrain = Jiyi::DECODE[0];
static const Jiyi::DecodeConfig &obstacle = Jiyi::DECODE[1];

static AP_HAL::CANFrame frame_from(uint32_t id, std::initializer_list<uint8_t> bytes)
{
    uint8_t data[8] {};
    uint8_t len = 0;
    for (const uint8_t b : bytes) {
        data[len++] = b;
    }
    return AP_HAL::CANFrame(id, data, len);
}

static Reason decode(const Jiyi::DecodeConfig &cfg, const AP_HAL::CANFrame &f,
                     float &d, uint8_t &s, float floor_m = 0, int32_t min_strength = -1)
{
    if (min_strength < 0) {
        min_strength = cfg.min_strength;
    }
    return Jiyi::decode(cfg, f, floor_m, min_strength, d, s);
}

TEST(JiyiCAN, DecodeTable)
{
    EXPECT_EQ(terrain.can_id, 214);
    EXPECT_EQ(obstacle.can_id, 220);
    EXPECT_FALSE(terrain.synth_clear);      // never a fabricated in-range distance
    EXPECT_TRUE(obstacle.synth_clear);
    EXPECT_EQ(terrain.near_rank, 0);
    EXPECT_EQ(obstacle.near_rank, 2);
}

// hand-computed frames, independent of the decoder's own checksum loop
TEST(JiyiCAN, ValidFrames)
{
    float d = -1;
    uint8_t s = 0;

    // terrain 500 cm, strength 200: chk = (01+F4+C8) & FF = BD
    EXPECT_EQ(decode(terrain, frame_from(214, {0xEA, 0x2D, 0x04, 0x00, 0x01, 0xF4, 0xC8, 0xBD}), d, s), Reason::OK);
    EXPECT_FLOAT_EQ(d, 5.0f);
    EXPECT_EQ(s, 200);

    // obstacle 1000 cm, strength 255: chk = (D3+3F+04+01+03+E8+FF) & FF = 01
    EXPECT_EQ(decode(obstacle, frame_from(220, {0xD3, 0x3F, 0x04, 0x01, 0x03, 0xE8, 0xFF, 0x01}), d, s), Reason::OK);
    EXPECT_FLOAT_EQ(d, 10.0f);
    EXPECT_EQ(s, 255);

    // big-endian: 0x0102 = 258 cm; chk = (01+02+64) & FF = 67
    EXPECT_EQ(decode(terrain, frame_from(214, {0xEA, 0x2D, 0x04, 0x00, 0x01, 0x02, 0x64, 0x67}), d, s), Reason::OK);
    EXPECT_FLOAT_EQ(d, 2.58f);
}

TEST(JiyiCAN, ChecksumCoverageDiffersPerSensor)
{
    float d;
    uint8_t s;
    // obstacle frame carrying a terrain-style checksum (03+E8+FF = EA)
    EXPECT_EQ(decode(obstacle, frame_from(220, {0xD3, 0x3F, 0x04, 0x01, 0x03, 0xE8, 0xFF, 0xEA}), d, s), Reason::CHK);
    // terrain frame carrying an obstacle-style checksum (EA+2D+04+00+01+F4+C8 = B8)
    EXPECT_EQ(decode(terrain, frame_from(214, {0xEA, 0x2D, 0x04, 0x00, 0x01, 0xF4, 0xC8, 0xB8}), d, s), Reason::CHK);
}

TEST(JiyiCAN, InvalidFrames)
{
    float d;
    uint8_t s;
    // short frame
    EXPECT_EQ(decode(terrain, frame_from(214, {0xEA, 0x2D, 0x04, 0x00, 0x01, 0xF4, 0xC8}), d, s), Reason::DLC);
    // obstacle header seen by the terrain decoder
    EXPECT_EQ(decode(terrain, frame_from(214, {0xD3, 0x3F, 0x04, 0x01, 0x03, 0xE8, 0xFF, 0x01}), d, s), Reason::HDR);
}

TEST(JiyiCAN, NoTargetIsDistinctFromRejects)
{
    float d;
    uint8_t s;
    EXPECT_EQ(decode(terrain, frame_from(214, {0xEA, 0x2D, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00}), d, s), Reason::NOTGT);
    // chk = (D3+3F+04+01+00+00+10) & FF = 27
    EXPECT_EQ(decode(obstacle, frame_from(220, {0xD3, 0x3F, 0x04, 0x01, 0x00, 0x00, 0x10, 0x27}), d, s), Reason::NOTGT);
}

TEST(JiyiCAN, Filters)
{
    float d;
    uint8_t s;
    // 1 cm is below sane_min_m (0.02): chk = (00+01+C8) & FF = C9
    EXPECT_EQ(decode(terrain, frame_from(214, {0xEA, 0x2D, 0x04, 0x00, 0x00, 0x01, 0xC8, 0xC9}), d, s), Reason::RANGE);
    // 5001 cm is above sane_max_m (50): 0x1389, chk = (13+89+C8) & FF = 64
    EXPECT_EQ(decode(terrain, frame_from(214, {0xEA, 0x2D, 0x04, 0x00, 0x13, 0x89, 0xC8, 0x64}), d, s), Reason::RANGE);
    // 5 m under a 6 m floor
    EXPECT_EQ(decode(terrain, frame_from(214, {0xEA, 0x2D, 0x04, 0x00, 0x01, 0xF4, 0xC8, 0xBD}), d, s, 6.0f), Reason::FLOOR);

    // obstacle strength gate is "strength < 25": 25 passes, 24 is weak
    // 1000 cm strength 25: chk = (D3+3F+04+01+03+E8+19) & FF = 1B
    EXPECT_EQ(decode(obstacle, frame_from(220, {0xD3, 0x3F, 0x04, 0x01, 0x03, 0xE8, 0x19, 0x1B}), d, s), Reason::OK);
    EXPECT_EQ(decode(obstacle, frame_from(220, {0xD3, 0x3F, 0x04, 0x01, 0x03, 0xE8, 0x18, 0x1A}), d, s), Reason::LOW_STRENGTH);
    EXPECT_EQ(s, 0x18);

    // terrain has no strength gate
    // 500 cm strength 0: chk = (01+F4+00) & FF = F5
    EXPECT_EQ(decode(terrain, frame_from(214, {0xEA, 0x2D, 0x04, 0x00, 0x01, 0xF4, 0x00, 0xF5}), d, s), Reason::OK);
}

TEST(JiyiCAN, ObstacleSelectsSecondSmallest)
{
    // one near spike is rejected
    const float spike[] {10, 10, 10, 3, 10};
    EXPECT_FLOAT_EQ(Jiyi::select_filtered(spike, 5, obstacle.near_rank), 10);
    // two near returns win immediately, where a plain median would still say 10
    const float closing[] {10, 3, 10, 3.2f, 10};
    EXPECT_FLOAT_EQ(Jiyi::select_filtered(closing, 5, obstacle.near_rank), 3.2f);
    EXPECT_FLOAT_EQ(Jiyi::select_filtered(closing, 5, terrain.near_rank), 10);
}

TEST(JiyiCAN, TerrainPlainMedian)
{
    const float five[] {5, 1, 9, 3, 7};
    EXPECT_FLOAT_EQ(Jiyi::select_filtered(five, 5, 0), 5);
    const float three[] {3, 1, 2};
    EXPECT_FLOAT_EQ(Jiyi::select_filtered(three, 3, 0), 2);
    // even length takes the lower middle, as Lua's t[(#t + 1) // 2]
    const float four[] {4, 1, 3, 2};
    EXPECT_FLOAT_EQ(Jiyi::select_filtered(four, 4, 0), 2);
}

#endif  // AP_RANGEFINDER_JIYI_CAN_ENABLED

AP_GTEST_MAIN()
