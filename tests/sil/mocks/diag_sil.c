/**
 * diag_sil.c  –  Implementación REAL de Diag_Log para build SIL
 *
 * Sobreescribe la versión __attribute__((weak)) de diag.c.
 * Escribe cada línea:
 *   1. A stdout (consola) con colores ANSI opcionales.
 *   2. Al fichero de log establecido con SIL_DiagSetFile().
 *
 * FORMATO DE SALIDA:
 *   [PASS] S1_STATE_MACHINE :: 1.1_mutex_handle_valid
 *   [FAIL] S6_CONTROL       :: 6.2_25pct_throttle  got=0 range=[1,35]
 *   ...
 *   === INFORME FINAL ===
 */

#include "diag.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>

/* Fichero de log activo (NULL = solo stdout) */
static FILE *s_log_file = NULL;

/**
 * @brief  Establece el fichero de log donde se escribirán los resultados.
 *         Llamar antes de Test_IntegrationRunAll().
 * @param  f  Puntero FILE abierto en modo escritura, o NULL para solo stdout.
 */
void SIL_DiagSetFile(FILE *f)
{
    s_log_file = f;
}

/* -------------------------------------------------------------------------
   Diag_Log  –  implementación fuerte (sobreescribe el weak de diag.c)
   ---------------------------------------------------------------------- */
void Diag_Log(const char *fmt, ...)
{
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    /* ---- Colorear según prefijo ---- */
    const char *color_start = "";
    const char *color_end   = "";

#ifdef SIL_ANSI_COLORS
    if      (strncmp(buf, "[PASS]",    6) == 0) { color_start = "\033[32m"; color_end = "\033[0m"; }  /* verde  */
    else if (strncmp(buf, "[FAIL]",    6) == 0) { color_start = "\033[31m"; color_end = "\033[0m"; }  /* rojo   */
    else if (strncmp(buf, "[SUITE",    6) == 0) { color_start = "\033[36m"; color_end = "\033[0m"; }  /* cyan   */
    else if (strncmp(buf, "===",       3) == 0 ||
             strncmp(buf, "---",       3) == 0) { color_start = "\033[33m"; color_end = "\033[0m"; }  /* amarillo */
    else if (strncmp(buf, "  RESULT",  8) == 0) { color_start = "\033[1m";  color_end = "\033[0m"; }  /* negrita */
#endif

    /* ---- Stdout ---- */
    printf("%s%s%s\n", color_start, buf, color_end);
    fflush(stdout);

    /* ---- Fichero de log ---- */
    if (s_log_file) {
        fprintf(s_log_file, "%s\n", buf);   /* sin colores ANSI en el fichero */
        fflush(s_log_file);
    }
}

/* -------------------------------------------------------------------------
   Diag_Report  –  stub (en SIL no hay colas de diagnóstico reales)
   ---------------------------------------------------------------------- */
void Diag_Report(osMessageQueueId_t rxQ, osMessageQueueId_t txQ)
{
    (void)rxQ; (void)txQ;
}
