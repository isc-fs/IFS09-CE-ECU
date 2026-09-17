/**
 * sil_boot_sequence.h
 * Boot sequence verification for SIL
 */

#ifndef SIL_BOOT_SEQUENCE_H
#define SIL_BOOT_SEQUENCE_H

#include <stdint.h>

/* Boot sequence states */
typedef enum {
    BOOT_STATE_INITIAL,
    BOOT_STATE_PRECHARGE_WAIT,
    BOOT_STATE_PRECHARGE_OK,
    BOOT_STATE_READY,
    BOOT_STATE_ERROR
} boot_state_t;

/* Get current boot state */
boot_state_t SIL_GetBootState(void);

/* Check if boot sequence completed successfully */
int SIL_IsBootComplete(void);

/* Print boot sequence status */
void SIL_PrintBootStatus(void);

#endif /* SIL_BOOT_SEQUENCE_H */
