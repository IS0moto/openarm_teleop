#include <gtest/gtest.h>
#include <cstring>
#include "openarm_wifi_teleop/safety/bilateral_gate.hpp"

using namespace openarm_wifi_teleop;

static utils::LinkInfo wired_link() {
    utils::LinkInfo l;
    l.resolved = true;
    l.ifname = "eth0";
    l.local_ip = "192.168.2.102";
    l.has_device = true;
    l.carrier = true;
    l.speed_mbps = 1000;
    return l;
}

static core::FeedbackStateBuffer::Stats good_stats() {
    core::FeedbackStateBuffer::Stats s;
    s.has_data = true;
    s.age_ms = 2.0;
    s.rate_hz = 498.0;
    s.loss_percent = 0.0;
    s.rtt_p50_ms = 3.0;
    s.rtt_p95_ms = 4.5;
    s.rtt_max_ms = 6.0;
    return s;
}

static net::FeedbackPacket good_packet() {
    net::FeedbackPacket p;
    std::memset(&p, 0, sizeof(p));
    p.link_ok = 1;
    p.enabled = 1;
    return p;
}

TEST(BilateralGateTest, WiredFastLinkPasses) {
    safety::BilateralGate gate(safety::BilateralGateConfig{});
    EXPECT_TRUE(gate.check_link(wired_link()).ok);
}

TEST(BilateralGateTest, WirelessRejected) {
    safety::BilateralGate gate(safety::BilateralGateConfig{});
    auto l = wired_link();
    l.ifname = "wlan0";
    l.is_wireless = true;
    l.speed_mbps = -1;
    auto r = gate.check_link(l);
    EXPECT_FALSE(r.ok);
    EXPECT_NE(r.reasons_joined().find("wireless"), std::string::npos);
}

TEST(BilateralGateTest, SlowLinkRejected) {
    safety::BilateralGate gate(safety::BilateralGateConfig{});
    auto l = wired_link();
    l.speed_mbps = 10;
    EXPECT_FALSE(gate.check_link(l).ok);
}

TEST(BilateralGateTest, LoopbackNeedsOptIn) {
    utils::LinkInfo l;
    l.resolved = true;
    l.ifname = "lo";
    l.is_loopback = true;
    l.carrier = true;
    safety::BilateralGateConfig cfg;
    EXPECT_FALSE(safety::BilateralGate(cfg).check_link(l).ok);
    cfg.allow_loopback = true;
    EXPECT_TRUE(safety::BilateralGate(cfg).check_link(l).ok);
}

TEST(BilateralGateTest, UnresolvedRejected) {
    utils::LinkInfo l;
    l.error = "no route";
    EXPECT_FALSE(safety::BilateralGate(safety::BilateralGateConfig{}).check_link(l).ok);
}

TEST(BilateralGateTest, GoodFeedbackPasses) {
    safety::BilateralGate gate(safety::BilateralGateConfig{});
    auto p = good_packet();
    EXPECT_TRUE(gate.check_feedback("right", good_stats(), &p).ok);
}

TEST(BilateralGateTest, NoFeedbackRejected) {
    safety::BilateralGate gate(safety::BilateralGateConfig{});
    core::FeedbackStateBuffer::Stats s;
    EXPECT_FALSE(gate.check_feedback("right", s, nullptr).ok);
}

TEST(BilateralGateTest, HighRttRejected) {
    safety::BilateralGate gate(safety::BilateralGateConfig{});
    auto s = good_stats();
    s.rtt_p95_ms = 12.0;
    auto p = good_packet();
    auto r = gate.check_feedback("right", s, &p);
    EXPECT_FALSE(r.ok);
    EXPECT_NE(r.reasons_joined().find("rtt"), std::string::npos);
}

TEST(BilateralGateTest, LowRateRejected) {
    safety::BilateralGate gate(safety::BilateralGateConfig{});
    auto s = good_stats();
    s.rate_hz = 100.0;
    auto p = good_packet();
    EXPECT_FALSE(gate.check_feedback("right", s, &p).ok);
}

TEST(BilateralGateTest, LossRejected) {
    safety::BilateralGate gate(safety::BilateralGateConfig{});
    auto s = good_stats();
    s.loss_percent = 5.0;
    auto p = good_packet();
    EXPECT_FALSE(gate.check_feedback("right", s, &p).ok);
}

TEST(BilateralGateTest, FollowerLinkNotOkRejected) {
    safety::BilateralGate gate(safety::BilateralGateConfig{});
    auto p = good_packet();
    p.link_ok = 0;
    EXPECT_FALSE(gate.check_feedback("right", good_stats(), &p).ok);
}

TEST(BilateralGateTest, MissingRttSamplesRejected) {
    safety::BilateralGate gate(safety::BilateralGateConfig{});
    auto s = good_stats();
    s.rtt_p95_ms = -1.0;
    auto p = good_packet();
    EXPECT_FALSE(gate.check_feedback("right", s, &p).ok);
}
