/**
 * sil_results.h
 * Results logging for SIL simulation
 */

#ifndef SIL_RESULTS_H
#define SIL_RESULTS_H

#include <stdio.h>
#include <stdint.h>
#include <time.h>

/* Ensure the results directory exists */
int SIL_EnsureResultsDir(void);

/* Initialize results file */
void SIL_Results_Init(const char *filename);

/* Log test result */
void SIL_Results_Log(const char *test_name, const char *result, const char *details);

/* Log timestamped event */
void SIL_Results_LogEvent(uint32_t timestamp_ms, const char *event, const char *data);

/* Close results file */
void SIL_Results_Close(void);

/* Get path to results directory */
const char* SIL_GetResultsPath(void);

#endif /* SIL_RESULTS_H */
