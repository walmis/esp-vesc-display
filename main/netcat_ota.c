#include <FreeRTOS.h>
#include "task.h"
#include <stdint.h>
#include <sys/socket.h>
#include <esp_ota_ops.h>
#include <netinet/in.h>
#include "esp_log.h"

#include <string.h>
#include <strings.h>
#include <stdio.h>
#include "esp_system.h"
#include "sdkconfig.h"

#define TAG "netcat-ota"

#ifdef CONFIG_DISPLAY_ROTATION
#include "display.h"
#define SHOW_MESSAGE(fmt, ...) { char* msg; asprintf(&msg, fmt, ##__VA_ARGS__); display_show_message(msg); free(msg); }
#else 
#define SHOW_MESSAGE(fmt, ...)
#endif

static esp_ota_handle_t update_handle;
static const esp_partition_t *update_part;
static const esp_partition_t *configured_part;
static const esp_partition_t *running_part;

void netcat_ota_task(void* arg) {
    uint16_t port = (uint16_t)arg;

    int sock = socket(AF_INET, SOCK_STREAM, 0);

	configured_part = esp_ota_get_boot_partition();
	running_part = esp_ota_get_running_partition();

    struct sockaddr_in saddr;
	saddr.sin_addr.s_addr = 0;
	saddr.sin_port = ntohs(port);
	saddr.sin_family = AF_INET;

	bind(sock, (struct sockaddr*)&saddr, sizeof(saddr));
	listen(sock, 1);

    while(1) {
        int client = accept(sock, 0, 0);
        update_part = esp_ota_get_next_update_partition(NULL);
        if (update_part == NULL)
        {
            ESP_LOGE(TAG, "update_partition not found!");
            close(client);
            continue;
        }

        SHOW_MESSAGE("OTA erasing...");
        vTaskDelay(10);

        err_t err = esp_ota_begin(update_part, OTA_SIZE_UNKNOWN, &update_handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_begin failed, error=%d", err);
            close(client);
            continue;
        }

        size_t total_bytes = 0;

        uint8_t* buffer = malloc(2048);
        int ret;
        do {
            ret = recv(client, buffer, 2048, 0);
            if(ret > 0) {
                total_bytes += ret;
                //vTaskDelay(1);

                esp_ota_write(update_handle, buffer, ret);
                SHOW_MESSAGE("OTA written %d bytes", total_bytes);
                //taskYIELD();
            }
        } while(ret > 0);

        err = esp_ota_end(update_handle);
        if(err == ESP_OK) {
            err = esp_ota_set_boot_partition(update_part);
			if (err != ESP_OK) {
				ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
                SHOW_MESSAGE("OTA Failed");
			} else {
                SHOW_MESSAGE("OTA OK!");
                vTaskDelay(10);
                esp_restart();
            }
        } else {
            SHOW_MESSAGE("OTA Failed");
        }
        close(client);


        free(buffer);

    }

}