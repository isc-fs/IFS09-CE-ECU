#ifndef ECU_DIAG_H
#define ECU_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

static inline void Diag_Log(const char *fmt, ...)
{
  (void)fmt;
}

#ifdef __cplusplus
}
#endif

#endif /* ECU_DIAG_H */
