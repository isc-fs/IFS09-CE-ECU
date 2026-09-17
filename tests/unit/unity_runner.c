#include "unity.h"
#include <stdio.h>

/* Forward declarations de los test groups */
extern void setUp(void);
extern void tearDown(void);

extern void TEST_GROUP_RUNNER(CAN_Parsing)(void);
extern void TEST_GROUP_RUNNER(CAN_Packing)(void);
extern void TEST_GROUP_RUNNER(ControlLogic)(void);
extern void TEST_GROUP_RUNNER(Telemetry)(void);
extern void TEST_GROUP_RUNNER(AppState)(void);
extern void TEST_GROUP_RUNNER(ErrorHandling)(void);

int main(void)
{
    UNITY_BEGIN();
    
    printf("\n========== CAN PARSING TESTS ==========\n");
    TEST_GROUP_RUNNER(CAN_Parsing)();
    
    printf("\n========== CAN PACKING TESTS ==========\n");
    TEST_GROUP_RUNNER(CAN_Packing)();
    
    printf("\n========== CONTROL LOGIC TESTS ==========\n");
    TEST_GROUP_RUNNER(ControlLogic)();
    
    printf("\n========== TELEMETRY TESTS ==========\n");
    TEST_GROUP_RUNNER(Telemetry)();
    
    printf("\n========== APP STATE TESTS ==========\n");
    TEST_GROUP_RUNNER(AppState)();
    
    printf("\n========== ERROR HANDLING TESTS ==========\n");
    TEST_GROUP_RUNNER(ErrorHandling)();
    
    return UNITY_END();
}
