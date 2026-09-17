/**
 * sil_results.c
 * Results logging implementation
 */

#include "sil_results.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <direct.h>
#define SIL_MKDIR(path) _mkdir(path)
#else
#include <sys/stat.h>
#include <sys/types.h>
#define SIL_MKDIR(path) mkdir(path, 0777)
#endif

static FILE *results_file = NULL;
static char results_path[256] = {0};

static int sil_mkdir_if_needed(const char *path)
{
    if (SIL_MKDIR(path) == 0 || errno == EEXIST) {
        return 0;
    }

    return -1;
}

int SIL_EnsureResultsDir(void)
{
    if (sil_mkdir_if_needed("tests") != 0) {
        return -1;
    }

    if (sil_mkdir_if_needed("tests/sil") != 0) {
        return -1;
    }

    if (sil_mkdir_if_needed("tests/sil/results") != 0) {
        return -1;
    }

    return 0;
}

void SIL_Results_Init(const char *filename)
{
    snprintf(results_path, sizeof(results_path), "tests/sil/results/%s", filename);

    if (SIL_EnsureResultsDir() != 0) {
        printf("[ERROR] Could not ensure results directory exists: tests/sil/results\n");
        return;
    }

    results_file = fopen(results_path, "w");
    if (!results_file) {
        printf("[ERROR] Could not open results file: %s\n", results_path);
        return;
    }
    
    fprintf(results_file, "╔════════════════════════════════════════════════════════════╗\n");
    fprintf(results_file, "║         ECU08 NSIL - Simulation Results                   ║\n");
    fprintf(results_file, "╚════════════════════════════════════════════════════════════╝\n\n");
    
    time_t now = time(NULL);
    fprintf(results_file, "Timestamp: %s\n", ctime(&now));
    fprintf(results_file, "\n════════════════════════════════════════════════════════════\n\n");
    
    fflush(results_file);
    printf("[RESULTS] Logging to: %s\n", results_path);
}

void SIL_Results_Log(const char *test_name, const char *result, const char *details)
{
    if (!results_file) return;
    
    fprintf(results_file, "[%s] %s\n", test_name, result);
    if (details) {
        fprintf(results_file, "  Details: %s\n", details);
    }
    fprintf(results_file, "\n");
    fflush(results_file);
}

void SIL_Results_LogEvent(uint32_t timestamp_ms, const char *event, const char *data)
{
    if (!results_file) return;
    
    fprintf(results_file, "[%5u ms] %s", timestamp_ms, event);
    if (data) {
        fprintf(results_file, ": %s", data);
    }
    fprintf(results_file, "\n");
    fflush(results_file);
}

void SIL_Results_Close(void)
{
    if (!results_file) return;
    
    fprintf(results_file, "\n════════════════════════════════════════════════════════════\n");
    fprintf(results_file, "Simulation complete\n");
    fclose(results_file);
    printf("[RESULTS] Results saved to: %s\n", results_path);
}

const char* SIL_GetResultsPath(void)
{
    return results_path;
}
