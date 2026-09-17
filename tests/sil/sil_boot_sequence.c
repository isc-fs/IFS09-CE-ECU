/**
 * sil_boot_sequence.c
 * Boot sequence tracking
 */

#include "sil_boot_sequence.h"
#include <stdio.h>

static boot_state_t current_boot_state = BOOT_STATE_INITIAL;

boot_state_t SIL_GetBootState(void)
{
    return current_boot_state;
}

int SIL_IsBootComplete(void)
{
    return current_boot_state == BOOT_STATE_READY;
}

void SIL_PrintBootStatus(void)
{
    const char *state_names[] = {
        "INITIAL",
        "PRECHARGE_WAIT",
        "PRECHARGE_OK",
        "READY",
        "ERROR"
    };
    
    if (current_boot_state < sizeof(state_names) / sizeof(state_names[0])) {
        printf("[BOOT] Current state: %s\n", state_names[current_boot_state]);
    }
}
