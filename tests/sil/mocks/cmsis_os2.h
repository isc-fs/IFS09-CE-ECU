/**
 * cmsis_os2.h  –  Mock CMSIS-RTOS v2 para entorno SIL (host PC)
 *
 * Proporciona los tipos y prototipos necesarios para compilar los módulos
 * de aplicación (app_state, control, can, telemetry, test_integration)
 * fuera del STM32 / FreeRTOS real.
 *
 * En el build SIL, el directorio mocks/ va PRIMERO en el include path,
 * por lo que este fichero sobreescribe el cmsis_os2.h real del middleware.
 */

#ifndef CMSIS_OS2_SIL_H_
#define CMSIS_OS2_SIL_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
   Tipos de estado
   ---------------------------------------------------------------------- */
typedef enum {
    osOK                = 0,
    osError             = -1,
    osErrorTimeout      = -2,
    osErrorResource     = -3,
    osErrorParameter    = -4,
    osErrorNoMemory     = -8,
    osErrorISR          = -16,
    osStatusReserved    = 0x7FFFFFFF
} osStatus_t;

/* -------------------------------------------------------------------------
   Constantes
   ---------------------------------------------------------------------- */
#define osWaitForever   0xFFFFFFFFU

/* -------------------------------------------------------------------------
   Prioridades de tarea (solo definiciones, no se usan en SIL)
   ---------------------------------------------------------------------- */
typedef int32_t osPriority_t;
#define osPriorityLow       (-2)
#define osPriorityBelowNormal (-1)
#define osPriorityNormal    (0)
#define osPriorityAboveNormal (1)
#define osPriorityHigh      (2)

/* -------------------------------------------------------------------------
   Handles opacos
   ---------------------------------------------------------------------- */
typedef void *osMutexId_t;
typedef void *osMessageQueueId_t;
typedef void *osThreadId_t;
typedef void *osSemaphoreId_t;

/* -------------------------------------------------------------------------
   Atributos (usados solo como punteros, ignorados en SIL)
   ---------------------------------------------------------------------- */
typedef struct { const char *name; uint32_t attr_bits; void *cb_mem; uint32_t cb_size; } osMutexAttr_t;
typedef struct { const char *name; uint32_t attr_bits; void *cb_mem; uint32_t cb_size; void *mq_mem; uint32_t mq_size; } osMessageQueueAttr_t;
typedef struct { const char *name; uint32_t attr_bits; void *cb_mem; uint32_t cb_size; void *stack_mem; uint32_t stack_size; osPriority_t priority; } osThreadAttr_t;

/* -------------------------------------------------------------------------
   API de Kernel / Timing
   ---------------------------------------------------------------------- */
uint32_t osKernelGetTickCount(void);
uint32_t osKernelGetTickFreq(void);
osStatus_t osKernelInitialize(void);
osStatus_t osKernelStart(void);

/* -------------------------------------------------------------------------
   API de Threads (stubs – SIL es single-threaded)
   ---------------------------------------------------------------------- */
void osDelay(uint32_t ticks);
void osDelayUntil(uint32_t ticks);
void osThreadExit(void);
osStatus_t osThreadSuspend(osThreadId_t thread_id);
osThreadId_t osThreadNew(void (*func)(void *), void *argument, const osThreadAttr_t *attr);

/* -------------------------------------------------------------------------
   API de Mutex (no-op en single-threaded SIL)
   ---------------------------------------------------------------------- */
osMutexId_t  osMutexNew(const osMutexAttr_t *attr);
osStatus_t   osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout);
osStatus_t   osMutexRelease(osMutexId_t mutex_id);
osStatus_t   osMutexDelete(osMutexId_t mutex_id);

/* -------------------------------------------------------------------------
   API de Message Queue (ring buffer implementado en cmsis_os2_impl.c)
   ---------------------------------------------------------------------- */
osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size,
                                     const osMessageQueueAttr_t *attr);
osStatus_t  osMessageQueuePut(osMessageQueueId_t mq_id, const void *msg_ptr,
                               uint8_t msg_prio, uint32_t timeout);
osStatus_t  osMessageQueueGet(osMessageQueueId_t mq_id, void *msg_ptr,
                               uint8_t *msg_prio, uint32_t timeout);
uint32_t    osMessageQueueGetCount(osMessageQueueId_t mq_id);
uint32_t    osMessageQueueGetSpace(osMessageQueueId_t mq_id);
osStatus_t  osMessageQueueReset(osMessageQueueId_t mq_id);

/* -------------------------------------------------------------------------
   Función de reset del tick SIL (solo para tests)
   ---------------------------------------------------------------------- */
void SIL_RTOS_Init(void);
void SIL_RTOS_ResetKernel(void);
void SIL_RTOS_RunReadyThreads(void);
void SIL_ResetTick(void);
void SIL_AdvanceTick(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* CMSIS_OS2_SIL_H_ */
