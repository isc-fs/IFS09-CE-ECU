/**
 * cmsis_os2_impl.c - Mock CMSIS-RTOS v2 for SIL
 *
 * Provides:
 * - simulated kernel tick
 * - cooperative thread scheduler for SIL
 * - no-op mutexes for single-threaded execution
 * - ring-buffer message queues
 */

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "can.h"
#include "telemetry.h"
#include "sil_hal_mocks.h"
#include <stdlib.h>
#include <string.h>

#define SIL_MAX_THREADS 16u

typedef struct {
    uint8_t *data;
    uint32_t msg_size;
    uint32_t capacity;
    uint32_t head;
    uint32_t tail;
    uint32_t count;
} sil_queue_t;

typedef enum {
    SIL_THREAD_UNUSED = 0,
    SIL_THREAD_READY,
    SIL_THREAD_SLEEPING,
    SIL_THREAD_SUSPENDED,
    SIL_THREAD_RUNNING,
    SIL_THREAD_EXITED
} sil_thread_state_t;

typedef struct {
    void (*func)(void *);
    void *argument;
    const char *name;
    osPriority_t priority;
    uint32_t wake_tick;
    uint32_t creation_order;
    sil_thread_state_t state;
} sil_thread_t;

typedef struct { int dummy; } sil_mutex_t;

extern osMessageQueueId_t canRxQueueHandle;
extern osMessageQueueId_t canTxQueueHandle;
extern osMessageQueueId_t telemetryEventQueueHandle;
extern osMessageQueueId_t telemetryRadioQueueHandle;
extern osMessageQueueId_t telemetrySdQueueHandle;
extern osMessageQueueId_t telemetryDashQueueHandle;
extern osMutexId_t g_inMutex;

static uint32_t s_tick_ms = 0;
static uint8_t s_kernel_started = 0;
static sil_thread_t s_threads[SIL_MAX_THREADS];
static uint32_t s_next_creation_order = 0;
static sil_thread_t *s_current_thread = NULL;

static uint32_t sil_delay_target(uint32_t ticks)
{
    return s_tick_ms + (ticks == 0u ? 1u : ticks);
}

static void sil_queue_destroy(osMessageQueueId_t *handle)
{
    sil_queue_t *q = handle ? (sil_queue_t *)(*handle) : NULL;
    if (!q) return;
    free(q->data);
    free(q);
    *handle = NULL;
}

static void sil_promote_woken_threads(void)
{
    uint32_t i;

    for (i = 0; i < SIL_MAX_THREADS; i++) {
        if (s_threads[i].state == SIL_THREAD_SLEEPING &&
            s_threads[i].wake_tick <= s_tick_ms) {
            s_threads[i].state = SIL_THREAD_READY;
        }
    }
}

static sil_thread_t *sil_pick_next_ready_thread(void)
{
    sil_thread_t *best = NULL;
    uint32_t i;

    sil_promote_woken_threads();

    for (i = 0; i < SIL_MAX_THREADS; i++) {
        sil_thread_t *thread = &s_threads[i];

        if (thread->state != SIL_THREAD_READY) continue;

        if (!best ||
            thread->priority > best->priority ||
            (thread->priority == best->priority &&
             thread->creation_order < best->creation_order)) {
            best = thread;
        }
    }

    return best;
}

extern uint32_t sil_get_time_ms(void);   /* harness clock, advanced by the SIL test loops */
uint32_t osKernelGetTickCount(void)
{
    /* The fixture-based harnesses advance sil_get_time_ms(), not this mock's
     * s_tick_ms (they call Control_Step10ms() directly, no osDelay). Return
     * whichever clock is live so the firmware's tick-based logic sees real
     * simulated time instead of a frozen 0. */
    uint32_t harness = sil_get_time_ms();
    return (harness > s_tick_ms) ? harness : s_tick_ms;
}

uint32_t osKernelGetTickFreq(void)
{
    return 1000u;
}

osStatus_t osKernelInitialize(void)
{
    s_tick_ms = 0;
    s_kernel_started = 0;
    return osOK;
}

osStatus_t osKernelStart(void)
{
    s_kernel_started = 1u;
    return osOK;
}

void osDelay(uint32_t ticks)
{
    if (!s_current_thread) {
        s_tick_ms += ticks;
        return;
    }

    s_current_thread->wake_tick = sil_delay_target(ticks);
    s_current_thread->state = SIL_THREAD_SLEEPING;
}

void osDelayUntil(uint32_t ticks)
{
    if (!s_current_thread) {
        if (ticks > s_tick_ms) s_tick_ms = ticks;
        return;
    }

    s_current_thread->wake_tick = (ticks > s_tick_ms) ? ticks : (s_tick_ms + 1u);
    s_current_thread->state = SIL_THREAD_SLEEPING;
}

void osThreadExit(void)
{
    if (s_current_thread) {
        s_current_thread->state = SIL_THREAD_EXITED;
    }
}

osStatus_t osThreadSuspend(osThreadId_t thread_id)
{
    sil_thread_t *thread = thread_id ? (sil_thread_t *)thread_id : s_current_thread;

    if (!thread) return osErrorParameter;

    thread->state = SIL_THREAD_SUSPENDED;
    return osOK;
}

osThreadId_t osThreadNew(void (*func)(void *), void *argument, const osThreadAttr_t *attr)
{
    uint32_t i;

    if (!func) return NULL;

    for (i = 0; i < SIL_MAX_THREADS; i++) {
        sil_thread_t *thread = &s_threads[i];

        if (thread->state != SIL_THREAD_UNUSED && thread->state != SIL_THREAD_EXITED) continue;

        memset(thread, 0, sizeof(*thread));
        thread->func = func;
        thread->argument = argument;
        thread->name = (attr && attr->name) ? attr->name : "thread";
        thread->priority = attr ? attr->priority : osPriorityNormal;
        thread->wake_tick = s_tick_ms;
        thread->creation_order = s_next_creation_order++;
        thread->state = SIL_THREAD_READY;
        return (osThreadId_t)thread;
    }

    return NULL;
}

osMutexId_t osMutexNew(const osMutexAttr_t *attr)
{
    sil_mutex_t *m;

    (void)attr;
    m = (sil_mutex_t *)malloc(sizeof(sil_mutex_t));
    if (!m) return NULL;
    m->dummy = 0;
    return (osMutexId_t)m;
}

osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout)
{
    (void)mutex_id;
    (void)timeout;
    return osOK;
}

osStatus_t osMutexRelease(osMutexId_t mutex_id)
{
    (void)mutex_id;
    return osOK;
}

osStatus_t osMutexDelete(osMutexId_t mutex_id)
{
    if (mutex_id) free(mutex_id);
    return osOK;
}

osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size,
                                     const osMessageQueueAttr_t *attr)
{
    sil_queue_t *q;

    (void)attr;
    q = (sil_queue_t *)calloc(1, sizeof(sil_queue_t));
    if (!q) return NULL;

    q->data = (uint8_t *)calloc(msg_count, msg_size);
    if (!q->data) {
        free(q);
        return NULL;
    }

    q->msg_size = msg_size;
    q->capacity = msg_count;
    return (osMessageQueueId_t)q;
}

osStatus_t osMessageQueuePut(osMessageQueueId_t mq_id, const void *msg_ptr,
                             uint8_t msg_prio, uint32_t timeout)
{
    sil_queue_t *q = (sil_queue_t *)mq_id;

    (void)msg_prio;
    (void)timeout;

    if (!q || !msg_ptr) return osErrorParameter;
    if (q->count >= q->capacity) return osErrorResource;

    memcpy(q->data + (q->tail * q->msg_size), msg_ptr, q->msg_size);
    q->tail = (q->tail + 1u) % q->capacity;
    q->count++;
    return osOK;
}

osStatus_t osMessageQueueGet(osMessageQueueId_t mq_id, void *msg_ptr,
                             uint8_t *msg_prio, uint32_t timeout)
{
    sil_queue_t *q = (sil_queue_t *)mq_id;

    (void)msg_prio;
    (void)timeout;

    if (!q || !msg_ptr) return osErrorParameter;
    if (q->count == 0u) return osErrorResource;

    memcpy(msg_ptr, q->data + (q->head * q->msg_size), q->msg_size);
    q->head = (q->head + 1u) % q->capacity;
    q->count--;
    return osOK;
}

uint32_t osMessageQueueGetCount(osMessageQueueId_t mq_id)
{
    sil_queue_t *q = (sil_queue_t *)mq_id;
    return q ? q->count : 0u;
}

uint32_t osMessageQueueGetSpace(osMessageQueueId_t mq_id)
{
    sil_queue_t *q = (sil_queue_t *)mq_id;
    return q ? (q->capacity - q->count) : 0u;
}

osStatus_t osMessageQueueReset(osMessageQueueId_t mq_id)
{
    sil_queue_t *q = (sil_queue_t *)mq_id;

    if (!q) return osErrorParameter;

    q->head = 0u;
    q->tail = 0u;
    q->count = 0u;
    return osOK;
}

void SIL_RTOS_ResetKernel(void)
{
    sil_queue_destroy(&canRxQueueHandle);
    sil_queue_destroy(&canTxQueueHandle);
    sil_queue_destroy(&telemetryEventQueueHandle);
    sil_queue_destroy(&telemetryRadioQueueHandle);
    sil_queue_destroy(&telemetrySdQueueHandle);
    sil_queue_destroy(&telemetryDashQueueHandle);

    if (g_inMutex) {
        (void)osMutexDelete(g_inMutex);
        g_inMutex = NULL;
    }

    memset(s_threads, 0, sizeof(s_threads));
    s_next_creation_order = 0u;
    s_current_thread = NULL;
    s_tick_ms = 0u;
    s_kernel_started = 0u;
}

void SIL_RTOS_Init(void)
{
    SIL_RTOS_ResetKernel();
    (void)osKernelInitialize();

    canRxQueueHandle = osMessageQueueNew(128u, sizeof(can_qitem16_t), NULL);
    canTxQueueHandle = osMessageQueueNew(64u, sizeof(can_qitem16_t), NULL);
    telemetryEventQueueHandle = osMessageQueueNew(32u, sizeof(telemetry_event_t), NULL);
    telemetryRadioQueueHandle = osMessageQueueNew(64u, sizeof(telemetry_frame_t), NULL);
    telemetrySdQueueHandle = osMessageQueueNew(64u, sizeof(telemetry_frame_t), NULL);
    telemetryDashQueueHandle = osMessageQueueNew(64u, sizeof(telemetry_frame_t), NULL);
    g_inMutex = osMutexNew(NULL);

    SIL_FDCAN_Reset();
}

void SIL_ResetTick(void)
{
    s_tick_ms = 0u;
}

void SIL_AdvanceTick(uint32_t ms)
{
    s_tick_ms += ms;
}

void SIL_RTOS_RunReadyThreads(void)
{
    sil_thread_t *thread;

    if (!s_kernel_started) return;

    thread = sil_pick_next_ready_thread();
    while (thread) {
        s_current_thread = thread;
        thread->state = SIL_THREAD_RUNNING;
        thread->func(thread->argument);

        if (thread->state == SIL_THREAD_RUNNING) {
            thread->wake_tick = s_tick_ms + 1u;
            thread->state = SIL_THREAD_SLEEPING;
        }

        s_current_thread = NULL;
        thread = sil_pick_next_ready_thread();
    }
}

size_t xPortGetFreeHeapSize(void)
{
    return 64u * 1024u;
}

size_t xPortGetMinimumEverFreeHeapSize(void)
{
    return 48u * 1024u;
}

uint32_t xTaskGetSchedulerState(void)
{
    return s_kernel_started ? 1u : taskSCHEDULER_NOT_STARTED;
}

void xPortSysTickHandler(void)
{
    /* no-op in SIL */
}
