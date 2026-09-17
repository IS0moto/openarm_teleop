#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <controller/control.hpp>

// Control only stores the OpenArm/Dynamics pointers in its constructor, so a
// null arm is enough to exercise the pure-math helpers.
static Control make_leader() {
    auto state = std::make_shared<RobotSystemState>(7, 1);
    return Control(nullptr, nullptr, nullptr, state, 1.0 / 500.0, ROLE_LEADER, "right_arm", 7, 1);
}

TEST(ControlFriction, GripperUsesItsOwnVelocityNotOutOfBoundsMemory) {
    Control c = make_leader();
    std::vector<double> Kp(8, 0), Kd(8, 0);
    std::vector<double> Fc(8, 0.0), k(8, 1.0), Fv(8, 0.0), Fo(8, 0.0);
    Fv[7] = 2.0;  // gripper: friction = 2 * v
    c.SetParameter(Kp, Kd, Fc, k, Fv, Fo);

    std::vector<double> arm_vel(7, 100.0);   // large arm velocities must not leak into index 7
    std::vector<double> grip_vel(1, 0.5);    // one-element vector, as in unilateral_step
    std::vector<double> friction;
    c.ComputeAllFriction(arm_vel, grip_vel, friction);

    ASSERT_EQ(friction.size(), 8u);
    EXPECT_DOUBLE_EQ(friction[7], 1.0);
    for (int i = 0; i < 7; ++i) EXPECT_DOUBLE_EQ(friction[i], 0.0);
}

TEST(ControlFriction, LeaderEffortClampDefaults) {
    Control c = make_leader();
    EXPECT_DOUBLE_EQ(c.ClampLeaderEffort(0, 35.0), 20.0);
    EXPECT_DOUBLE_EQ(c.ClampLeaderEffort(0, -35.0), -20.0);
    EXPECT_DOUBLE_EQ(c.ClampLeaderEffort(7, 1e170), 2.0);   // the value the ASCII garbage produced
    EXPECT_DOUBLE_EQ(c.ClampLeaderEffort(7, 0.1), 0.1);
    EXPECT_DOUBLE_EQ(c.ClampLeaderEffort(7, std::numeric_limits<double>::quiet_NaN()), 0.0);
    EXPECT_DOUBLE_EQ(c.ClampLeaderEffort(7, std::numeric_limits<double>::infinity()), 0.0);
}

TEST(ControlFriction, LeaderEffortClampFromConfig) {
    Control c = make_leader();
    c.SetEffortLimits({5, 5, 5, 5, 5, 5, 5, 0.5});
    EXPECT_DOUBLE_EQ(c.ClampLeaderEffort(3, 9.0), 5.0);
    EXPECT_DOUBLE_EQ(c.ClampLeaderEffort(7, -3.0), -0.5);
    c.SetEffortLimits({});  // back to defaults
    EXPECT_DOUBLE_EQ(c.ClampLeaderEffort(7, -3.0), -2.0);
}
