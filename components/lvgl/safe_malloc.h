#include <stdlib.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "FreeRTOS.h"
#include "task.h"

static inline void* safe_malloc(size_t size) {
    void* ret;
    while((ret = malloc(size)) == 0) {
        vTaskDelay(1);
    }
    return ret;
}
