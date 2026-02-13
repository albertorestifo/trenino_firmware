#include <unity.h>
#include "bldc_manager.h"

void test_bldc_manager_init() {
    BLDCManager::init();
    TEST_ASSERT_EQUAL_UINT8(0, BLDCManager::getLeverCount());
}

void setUp() {
    BLDCManager::init();
}

void tearDown() {
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_bldc_manager_init);
    UNITY_END();
    return 0;
}
