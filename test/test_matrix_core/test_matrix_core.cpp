#include "../../src/matrix_core.cpp"
#include <unity.h>

using namespace Modules;

void setUp(void) {}
void tearDown(void) {}

void test_press_needs_threshold_samples()
{
    MatrixCore core(2, 2, 3);
    uint8_t idx;
    bool pressed;

    core.sample(0, 1, true);
    core.sample(0, 1, true);
    TEST_ASSERT_FALSE(core.pop(idx, pressed));

    core.sample(0, 1, true);
    TEST_ASSERT_TRUE(core.pop(idx, pressed));
    TEST_ASSERT_EQUAL_UINT8(1, idx);
    TEST_ASSERT_TRUE(pressed);
    TEST_ASSERT_FALSE(core.pop(idx, pressed));

    // Held: no repeat events
    core.sample(0, 1, true);
    core.sample(0, 1, true);
    core.sample(0, 1, true);
    TEST_ASSERT_FALSE(core.pop(idx, pressed));

    // Release also needs threshold samples
    core.sample(0, 1, false);
    core.sample(0, 1, false);
    TEST_ASSERT_FALSE(core.pop(idx, pressed));
    core.sample(0, 1, false);
    TEST_ASSERT_TRUE(core.pop(idx, pressed));
    TEST_ASSERT_FALSE(pressed);
}

void test_bounce_resets_count()
{
    MatrixCore core(2, 2, 3);
    uint8_t idx;
    bool pressed;

    core.sample(1, 1, true);
    core.sample(1, 1, true);
    core.sample(1, 1, false); // bounce resets counter
    core.sample(1, 1, true);
    core.sample(1, 1, true);
    TEST_ASSERT_FALSE(core.pop(idx, pressed));
    core.sample(1, 1, true);
    TEST_ASSERT_TRUE(core.pop(idx, pressed));
    TEST_ASSERT_EQUAL_UINT8(3, idx);
}

void test_queue_full_drops_oldest()
{
    MatrixCore core(4, 4, 1);
    uint8_t idx;
    bool pressed;

    // 10 distinct presses with threshold 1; queue holds EVENT_QUEUE_SIZE - 1
    for (uint8_t i = 0; i < 10; i++) {
        core.sample(i / 4, i % 4, true);
    }

    uint8_t count = 0;
    uint8_t first = 0xFF, last = 0xFF;
    while (core.pop(idx, pressed)) {
        if (count == 0) first = idx;
        last = idx;
        count++;
    }
    TEST_ASSERT_EQUAL_UINT8(MatrixCore::EVENT_QUEUE_SIZE - 1, count);
    TEST_ASSERT_EQUAL_UINT8(9, last);
    TEST_ASSERT_EQUAL_UINT8(10 - count, first); // oldest dropped
}

void test_pop_returns_row_col_index()
{
    MatrixCore core(3, 4, 1);
    uint8_t idx;
    bool pressed;

    core.sample(2, 3, true);
    TEST_ASSERT_TRUE(core.pop(idx, pressed));
    TEST_ASSERT_EQUAL_UINT8(2 * 4 + 3, idx);
    TEST_ASSERT_TRUE(pressed);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_press_needs_threshold_samples);
    RUN_TEST(test_bounce_resets_count);
    RUN_TEST(test_queue_full_drops_oldest);
    RUN_TEST(test_pop_returns_row_col_index);
    return UNITY_END();
}
