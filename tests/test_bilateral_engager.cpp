#include <gtest/gtest.h>
#include "openarm_wifi_teleop/control/bilateral_engager.hpp"

using openarm_wifi_teleop::control::BilateralEngager;
using State = BilateralEngager::State;

static BilateralEngager make() {
    BilateralEngager::Config c;
    c.ramp_up_s = 0.5;
    c.ramp_down_s = 0.1;
    return BilateralEngager(c);
}

TEST(BilateralEngager, WaitsForAlignmentThenRamps) {
    auto e = make();
    EXPECT_EQ(e.state(), State::OFF);
    e.update(true, false, 0.002);
    EXPECT_EQ(e.state(), State::WAIT_ALIGN);
    EXPECT_FALSE(e.active());
    for (int i = 0; i < 100; ++i) e.update(true, false, 0.002);
    EXPECT_EQ(e.state(), State::WAIT_ALIGN);  // never engages while misaligned
    e.update(true, true, 0.002);
    EXPECT_EQ(e.state(), State::RAMP_UP);
    double prev = 0.0;
    for (int i = 0; i < 260; ++i) {
        e.update(true, false, 0.002);  // alignment no longer required once ramping
        EXPECT_GE(e.scale(), prev);
        prev = e.scale();
    }
    EXPECT_EQ(e.state(), State::ENGAGED);
    EXPECT_DOUBLE_EQ(e.scale(), 1.0);
}

TEST(BilateralEngager, ReleaseRampsDownNeverSteps) {
    auto e = make();
    e.update(true, true, 0.002);
    for (int i = 0; i < 300; ++i) e.update(true, true, 0.002);
    ASSERT_TRUE(e.engaged());
    e.update(false, true, 0.002);
    EXPECT_EQ(e.state(), State::RAMP_DOWN);
    EXPECT_GT(e.scale(), 0.9);
    int steps = 0;
    while (e.active()) { e.update(false, true, 0.002); ++steps; }
    EXPECT_EQ(e.state(), State::OFF);
    EXPECT_NEAR(steps * 0.002, 0.1, 0.01);
}

TEST(BilateralEngager, ReRequestDuringRampDownReturnsToWaitAlign) {
    auto e = make();
    e.update(true, true, 0.002);
    for (int i = 0; i < 300; ++i) e.update(true, true, 0.002);
    e.update(false, true, 0.002);
    for (int i = 0; i < 10; ++i) e.update(true, true, 0.002);  // want again mid-ramp
    EXPECT_EQ(e.state(), State::RAMP_DOWN);                      // finishes the release first
    while (e.state() == State::RAMP_DOWN) e.update(true, true, 0.002);
    EXPECT_EQ(e.state(), State::WAIT_ALIGN);
}

TEST(BilateralEngager, ResetIsImmediate) {
    auto e = make();
    e.update(true, true, 0.002);  // OFF -> WAIT_ALIGN
    e.update(true, true, 0.002);  // WAIT_ALIGN -> RAMP_UP
    e.update(true, true, 0.1);    // ramping
    EXPECT_GT(e.scale(), 0.0);
    e.reset();
    EXPECT_EQ(e.state(), State::OFF);
    EXPECT_DOUBLE_EQ(e.scale(), 0.0);
}
