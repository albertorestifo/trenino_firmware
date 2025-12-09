#include "../../src/heartbeat.h"
#include <unity.h>

using namespace Heartbeat;

// Test heartbeat manager initialization
void test_heartbeat_init()
{
    HeartbeatManager hb(2000);

    TEST_ASSERT_EQUAL(2000, hb.getInterval());
    TEST_ASSERT_EQUAL(0, hb.getLastMessageTime());
}

// Test that heartbeat should be sent initially
void test_heartbeat_should_send_initially()
{
    HeartbeatManager hb(2000);

    // At time 0, no messages sent yet, but we're past the interval
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(0));

    // At time 2000ms, should send heartbeat
    TEST_ASSERT_TRUE(hb.shouldSendHeartbeat(2000));
}

// Test that heartbeat should not be sent immediately after a message
void test_heartbeat_not_sent_after_message()
{
    HeartbeatManager hb(2000);

    // Send a message at time 1000ms
    hb.notifyMessageSent(1000);

    // Should not send heartbeat immediately
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(1000));

    // Should not send heartbeat 1 second later (only 1s elapsed)
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(2000));

    // Should send heartbeat 2 seconds after the message (2s elapsed)
    TEST_ASSERT_TRUE(hb.shouldSendHeartbeat(3000));
}

// Test that heartbeat resets timer when sent
void test_heartbeat_resets_timer()
{
    HeartbeatManager hb(2000);

    // Send heartbeat at time 2000ms
    hb.markHeartbeatSent(2000);

    TEST_ASSERT_EQUAL(2000, hb.getLastMessageTime());

    // Should not send another heartbeat immediately
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(2000));

    // Should not send heartbeat 1 second later
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(3000));

    // Should send heartbeat 2 seconds after the last one
    TEST_ASSERT_TRUE(hb.shouldSendHeartbeat(4000));
}

// Test multiple messages prevent heartbeat
void test_multiple_messages_prevent_heartbeat()
{
    HeartbeatManager hb(2000);

    // Send messages every 500ms
    hb.notifyMessageSent(500);
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(500));

    hb.notifyMessageSent(1000);
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(1000));

    hb.notifyMessageSent(1500);
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(1500));

    hb.notifyMessageSent(2000);
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(2000));

    // No message sent for 2 seconds, should send heartbeat
    TEST_ASSERT_TRUE(hb.shouldSendHeartbeat(4000));
}

// Test heartbeat with different intervals
void test_heartbeat_different_intervals()
{
    // Test with 1 second interval
    HeartbeatManager hb1(1000);
    hb1.notifyMessageSent(0);
    TEST_ASSERT_FALSE(hb1.shouldSendHeartbeat(500));
    TEST_ASSERT_TRUE(hb1.shouldSendHeartbeat(1000));

    // Test with 5 second interval
    HeartbeatManager hb5(5000);
    hb5.notifyMessageSent(0);
    TEST_ASSERT_FALSE(hb5.shouldSendHeartbeat(4000));
    TEST_ASSERT_TRUE(hb5.shouldSendHeartbeat(5000));
}

// Test edge case: message sent exactly at interval boundary
void test_heartbeat_exact_boundary()
{
    HeartbeatManager hb(2000);

    hb.notifyMessageSent(1000);

    // At exactly 2000ms after message (3000ms total)
    TEST_ASSERT_TRUE(hb.shouldSendHeartbeat(3000));

    // Just before the boundary
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(2999));
}

// Test time wraparound (millis() overflow after ~49 days)
void test_heartbeat_time_wraparound()
{
    HeartbeatManager hb(2000);

    // Note: On 64-bit systems, unsigned long is 64-bit, so wraparound works differently
    // We'll test with values that work on both 32-bit and 64-bit systems

    // Simulate a large timestamp
    unsigned long large_time = 4000000000UL;
    hb.notifyMessageSent(large_time);

    // 1000ms later - should not send heartbeat
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(large_time + 1000));

    // 2000ms later - should send heartbeat
    TEST_ASSERT_TRUE(hb.shouldSendHeartbeat(large_time + 2000));

    // 3000ms later - should send heartbeat
    TEST_ASSERT_TRUE(hb.shouldSendHeartbeat(large_time + 3000));
}

// Test realistic scenario: sensor readings followed by idle period
void test_heartbeat_realistic_scenario()
{
    HeartbeatManager hb(2000);

    // Device starts, sends identity response
    hb.notifyMessageSent(100);

    // Sensor readings at 500ms intervals
    hb.notifyMessageSent(600);
    hb.notifyMessageSent(1100);
    hb.notifyMessageSent(1600);

    // Last sensor reading at 1600ms
    // At 2000ms, only 400ms since last message - no heartbeat
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(2000));

    // At 3600ms, 2000ms since last message - send heartbeat
    TEST_ASSERT_TRUE(hb.shouldSendHeartbeat(3600));

    // Heartbeat sent
    hb.markHeartbeatSent(3600);

    // Another heartbeat at 5600ms
    TEST_ASSERT_TRUE(hb.shouldSendHeartbeat(5600));

    // New sensor reading interrupts heartbeat cycle
    hb.notifyMessageSent(5800);

    // Next heartbeat should be at 7800ms, not 7600ms
    TEST_ASSERT_FALSE(hb.shouldSendHeartbeat(7600));
    TEST_ASSERT_TRUE(hb.shouldSendHeartbeat(7800));
}

// Test that notifyMessageSent and markHeartbeatSent have same effect
void test_heartbeat_notify_equivalence()
{
    HeartbeatManager hb1(2000);
    HeartbeatManager hb2(2000);

    hb1.notifyMessageSent(1000);
    hb2.markHeartbeatSent(1000);

    TEST_ASSERT_EQUAL(hb1.getLastMessageTime(), hb2.getLastMessageTime());
    TEST_ASSERT_EQUAL(hb1.shouldSendHeartbeat(2000), hb2.shouldSendHeartbeat(2000));
    TEST_ASSERT_EQUAL(hb1.shouldSendHeartbeat(3000), hb2.shouldSendHeartbeat(3000));
}

// Test update() method with callback
static int g_heartbeat_count = 0;
static void test_callback()
{
    g_heartbeat_count++;
}

void test_heartbeat_update_with_callback()
{
    g_heartbeat_count = 0;
    HeartbeatManager hb(2000, test_callback);

    // At time 0, no heartbeat should be sent
    hb.update(0);
    TEST_ASSERT_EQUAL(0, g_heartbeat_count);

    // At time 2000ms, heartbeat should be sent
    hb.update(2000);
    TEST_ASSERT_EQUAL(1, g_heartbeat_count);

    // Immediately after, no heartbeat
    hb.update(2000);
    TEST_ASSERT_EQUAL(1, g_heartbeat_count);

    // At time 4000ms, another heartbeat
    hb.update(4000);
    TEST_ASSERT_EQUAL(2, g_heartbeat_count);
}

void test_heartbeat_update_prevents_after_message()
{
    g_heartbeat_count = 0;
    HeartbeatManager hb(2000, test_callback);

    // Send a message at 1000ms
    hb.notifyMessageSent(1000);

    // At 2000ms, no heartbeat (only 1s since message)
    hb.update(2000);
    TEST_ASSERT_EQUAL(0, g_heartbeat_count);

    // At 3000ms, heartbeat should be sent (2s since message)
    hb.update(3000);
    TEST_ASSERT_EQUAL(1, g_heartbeat_count);
}

void test_heartbeat_update_without_callback()
{
    HeartbeatManager hb(2000);

    // Should not crash when callback is null
    hb.update(0);
    hb.update(2000);

    // Last message time should still be updated
    TEST_ASSERT_EQUAL(2000, hb.getLastMessageTime());
}

void test_heartbeat_set_callback()
{
    g_heartbeat_count = 0;
    HeartbeatManager hb(2000);

    // No callback initially
    hb.update(2000);
    TEST_ASSERT_EQUAL(0, g_heartbeat_count);

    // Set callback
    hb.setCallback(test_callback);

    // Reset last message time
    hb.notifyMessageSent(0);

    // Now callback should be called
    hb.update(2000);
    TEST_ASSERT_EQUAL(1, g_heartbeat_count);
}

// Main test runner
void setUp(void)
{
    // Set up code (runs before each test)
}

void tearDown(void)
{
    // Clean up code (runs after each test)
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_heartbeat_init);
    RUN_TEST(test_heartbeat_should_send_initially);
    RUN_TEST(test_heartbeat_not_sent_after_message);
    RUN_TEST(test_heartbeat_resets_timer);
    RUN_TEST(test_multiple_messages_prevent_heartbeat);
    RUN_TEST(test_heartbeat_different_intervals);
    RUN_TEST(test_heartbeat_exact_boundary);
    RUN_TEST(test_heartbeat_time_wraparound);
    RUN_TEST(test_heartbeat_realistic_scenario);
    RUN_TEST(test_heartbeat_notify_equivalence);
    RUN_TEST(test_heartbeat_update_with_callback);
    RUN_TEST(test_heartbeat_update_prevents_after_message);
    RUN_TEST(test_heartbeat_update_without_callback);
    RUN_TEST(test_heartbeat_set_callback);

    return UNITY_END();
}
