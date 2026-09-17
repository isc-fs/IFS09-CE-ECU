#ifndef FREERTOS_TASK_SIL_H_
#define FREERTOS_TASK_SIL_H_

#include <stddef.h>
#include <stdint.h>

size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);
uint32_t xTaskGetSchedulerState(void);
void xPortSysTickHandler(void);

#endif /* FREERTOS_TASK_SIL_H_ */
