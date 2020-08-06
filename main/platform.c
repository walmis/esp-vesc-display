/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <assert.h>
#include <sys/time.h>
#include <sys/unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include "dhcpserver/dhcpserver.h"
//#include <sysparam.h>
#include "http.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/api.h"
#include "lwip/tcp.h"

#include "rom/ets_sys.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp8266/uart_register.h"

#include <lwip/sockets.h>

#include "ota-tftp.h"

#include "vesc/packet.h"
#include "vesc/datatypes_5_2.h"
#include "vesc/confgenerator_5_2.h"
#include "vesc/buffer.h"
#include "mcdata.h"
#include <errno.h>
#include <string.h>
#include <math.h>
#include "utils.h"

#include "display.h"
#include "sdkconfig.h"

#define ADC_BRAKE_THRESHOLD 50

static nvs_handle g_nvs_handle;
static uint32_t t_last_packet;
static xQueueHandle uart_events_queue;
static xQueueHandle dbg_queue;

static uint8_t g_motor_armed;

static float g_max_watts = 500;
static uint16_t g_throttle_cal_min;
static uint16_t g_throttle_cal_max;
static uint8_t g_brake_controls_disabled;
static int g_vesc_sock;

struct mc_data mc_data;
static mc_configuration mcconf;
static bool have_mcconf;

static double stored_odo;
static int prev_odo;
static uint32_t t_odo_save;
static int g_initial_tach;

static float tach_to_km;
static float rpm_to_kmh;

static float g_cc_current_set;
static float g_cc_rpm_set;
static uint8_t g_cc_enabled;

static int g_tcp_serv_sock;
static int g_udp_serv_sock;
static int g_tcp_client_sock;
static struct sockaddr_in g_udp_peer_addr;
static uint8_t g_udp_connected;
uint32_t uart_tx_count;


const char*
platform_target_voltage(void) {
  static char voltage[16];
  int vdd = esp_wifi_get_vdd33();
  sprintf(voltage, "%dmV", vdd);
  return voltage;
}

uint32_t platform_time_ms(void) {
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

#define vTaskDelayMs(ms)	vTaskDelay((ms)/portTICK_PERIOD_MS)

void platform_delay(uint32_t ms) {
  vTaskDelayMs(ms);
}

int platform_hwversion(void) {
  return 0;
}


void dbg_task(void *parameters) {
	dbg_queue = xQueueCreate(1024, 1);

	while(1) {

		char tmp;
		if(xQueuePeek(dbg_queue, &tmp, 10)) {
			struct netbuf* nb = netbuf_new();

			int waiting = uxQueueMessagesWaiting(dbg_queue);
			char* mem = netbuf_alloc(nb, waiting);

			while(waiting--) {
				xQueueReceive(dbg_queue, mem, 0);
				http_debug_putc(*mem, *mem=='\n' ? 1 : 0);
				mem++;
			}
			//netconn_sendto(nc, nb, &ip, 6666);

			netbuf_delete(nb);
		}

	}
}

#define PKT_VESC 0
#define PKT_UDP 1
#define PKT_TCP 2

//whole packet received from tcp/udp, forward to vesc (handler 0)
void _net_pkt_handler(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, PKT_VESC);
}

void _net_pkt_tcp_snd_handler(unsigned char *data, unsigned int len) {\
	if(g_tcp_client_sock != 0) {
		int ret = send(g_tcp_client_sock, data, len, MSG_DONTWAIT);

	}
}
void _net_pkt_udp_snd_handler(unsigned char *data, unsigned int len) {
	if(g_udp_connected) {

		int ret = sendto(g_udp_serv_sock, data, len, MSG_DONTWAIT, (struct sockaddr*)&g_udp_peer_addr, sizeof(g_udp_peer_addr));
		if(ret < 0 && ret != -EAGAIN) {
			g_udp_connected = false;
		}
	}
}

void net_proxy_task(void* params) {

	g_tcp_serv_sock = socket(AF_INET, SOCK_STREAM, 0);
	g_udp_serv_sock = socket(AF_INET, SOCK_DGRAM, 0);
	g_tcp_client_sock = 0;

	int ret;

	struct sockaddr_in saddr;
	saddr.sin_addr.s_addr = 0;
	saddr.sin_port = ntohs(CONFIG_VESC_PR_TCP_PORT);
	saddr.sin_family = AF_INET;

	bind(g_tcp_serv_sock, (struct sockaddr*)&saddr, sizeof(saddr));
	listen(g_tcp_serv_sock, 1);

	saddr.sin_addr.s_addr = 0;
	saddr.sin_port = ntohs(CONFIG_VESC_PR_UDP_PORT);
	saddr.sin_family = AF_INET;
	bind(g_udp_serv_sock, (struct sockaddr*)&saddr, sizeof(saddr));
	
	fd_set fds;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 100000;

	while(1) {
		//packet_timerfunc();

		FD_ZERO(&fds);
		FD_SET(g_tcp_serv_sock, &fds);
		FD_SET(g_udp_serv_sock, &fds);
		if(g_tcp_client_sock)
			FD_SET(g_tcp_client_sock, &fds);

		int maxfd = MAX(g_tcp_serv_sock, MAX(g_udp_serv_sock, g_tcp_client_sock));

		if((ret = select(maxfd+1, &fds, NULL, NULL, &tv) > 0))
		{
			if(FD_ISSET(g_tcp_serv_sock, &fds)) {
				g_tcp_client_sock = accept(g_tcp_serv_sock, 0, 0);
				if(g_tcp_client_sock < 0) {
					ESP_LOGE(__func__, "accept() failed");
					g_tcp_client_sock = 0;
				} else {
					ESP_LOGI(__func__, "accepted tcp connection");

					int opt = 1; /* SO_KEEPALIVE */
			        setsockopt(g_tcp_client_sock, SOL_SOCKET, SO_KEEPALIVE, (void *)&opt, sizeof(opt));
			        opt = 3; /* s TCP_KEEPIDLE */
			        setsockopt(g_tcp_client_sock, IPPROTO_TCP, TCP_KEEPIDLE, (void*)&opt, sizeof(opt));
			        opt = 1; /* s TCP_KEEPINTVL */
			        setsockopt(g_tcp_client_sock, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&opt, sizeof(opt));
			        opt = 3; /* TCP_KEEPCNT */
			        setsockopt(g_tcp_client_sock, IPPROTO_TCP, TCP_KEEPCNT, (void *)&opt, sizeof(opt));
			        opt = 1;
			        setsockopt(g_tcp_client_sock, IPPROTO_TCP, TCP_NODELAY, (void *)&opt, sizeof(opt));

				}
			}
			uint8_t buf[128];

			if(FD_ISSET(g_udp_serv_sock, &fds)) {
				socklen_t slen = sizeof(g_udp_peer_addr);
				ret = recvfrom(g_udp_serv_sock, buf, sizeof(buf), 0, (struct sockaddr*)&g_udp_peer_addr, &slen);
				if(ret > 0) {
					g_udp_connected = true;
					for(int i = 0; i < ret; i++) {
						packet_process_byte(buf[i], PKT_UDP);
					}
				} else {
					ESP_LOGE(__func__, "udp recvfrom() failed");
				}
			}

			if(g_tcp_client_sock && FD_ISSET(g_tcp_client_sock, &fds)) {
				ret = recv(g_tcp_client_sock, buf, sizeof(buf), MSG_DONTWAIT);
				if(ret > 0) {
					for(int i = 0; i < ret; i++) {
						packet_process_byte(buf[i], PKT_TCP);
					}
				} else {
					ESP_LOGE(__func__, "tcp client recv() failed (%s)", strerror(errno));
					close(g_tcp_client_sock);
					g_tcp_client_sock = 0;
				}
			}
		}
	}
}

void vesc_send_mcconf_temp() {
	if(have_mcconf) {
		uint8_t store = 0;
		uint8_t forward_can = 0;
		uint8_t divide_by_controllers = 0;
		uint8_t ack = 0;

		uint8_t buffer[64];
		int idx = 0;
		buffer[idx++] = COMM_SET_MCCONF_TEMP;
		buffer[idx++] = store;
		buffer[idx++] = forward_can;
		buffer[idx++] = ack;
		buffer[idx++] = divide_by_controllers;
		
		buffer_append_float32_auto(buffer, mcconf.l_current_min_scale, &idx);
		buffer_append_float32_auto(buffer, mcconf.l_current_max_scale, &idx);
		buffer_append_float32_auto(buffer, mcconf.l_min_erpm, &idx);
		buffer_append_float32_auto(buffer, mcconf.l_max_erpm, &idx);
		buffer_append_float32_auto(buffer, mcconf.l_min_duty, &idx);
		buffer_append_float32_auto(buffer, mcconf.l_max_duty, &idx);
		buffer_append_float32_auto(buffer, mcconf.l_watt_min, &idx);
		buffer_append_float32_auto(buffer, mcconf.l_watt_max, &idx);
		buffer_append_float32_auto(buffer, mcconf.l_in_current_min, &idx);
		buffer_append_float32_auto(buffer, mcconf.l_in_current_max, &idx);	

		packet_send_packet(buffer, idx, PKT_VESC);
	}
}

bool vesc_update_power_level() {
	if(have_mcconf) {
		if(g_max_watts < 0) g_max_watts = 0;

		float Pval = utils_map(g_set_power_level, 0, 4, 0, g_max_watts);
		mcconf.l_watt_max = Pval;
		vesc_send_mcconf_temp();

		return true;
	}
	return false;
}

void debug_putc(char c, int flush) {
	if(dbg_queue) {
		xQueueSend(dbg_queue, &c, 0);
	}
}

void platform_set_baud(uint32_t baud) {
	uart_set_baudrate(0, baud);
	uart_set_baudrate(1, baud);
	nvs_set_u32(g_nvs_handle, "uartbaud", baud);
}



static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    /* For accessing reason codes in case of disconnection */
    system_event_info_t *info = &event->event_info;

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI("WIFI", "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        //xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI("WIFI", "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI("WIFI", "station:"MACSTR" leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGE("WIFI", "Disconnect reason : %d", info->disconnected.reason);
        if (info->disconnected.reason == WIFI_REASON_BASIC_RATE_NOT_SUPPORT) {
            /*Switch to 802.11 bgn mode */
            //esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCAL_11B | WIFI_PROTOCAL_11G | WIFI_PROTOCAL_11N);
        }
        esp_wifi_connect();
        //xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

#ifdef CONFIG_ESP_AP_MODE
void wifi_init_softap()
{

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .password = CONFIG_ESP_AP_PSK,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
			.channel = 9

        },
    };

    uint64_t chipid;
    esp_read_mac((uint8_t*)&chipid, ESP_MAC_WIFI_SOFTAP);

    wifi_config.ap.ssid_len = sprintf((char*)wifi_config.ap.ssid, CONFIG_ESP_AP_SSID "_%X", (uint32_t)chipid);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}
#endif

int putc_noop(int c) {
	return c;
}

int putc_remote(int c) {
	if(c == '\n')
		debug_putc('\r', 0);

	debug_putc(c, c=='\n' ? 1 : 0);
	return c;
}




static void vesc_packet_send_cb(unsigned char *data, unsigned int len) {
#ifdef CONFIG_ESP_VESC_UART
	uart_write_bytes(0, (const char*)data, len);
	uart_wait_tx_done(0, portMAX_DELAY);
#else
	if(g_vesc_sock != 0) {
		int ret = send(g_vesc_sock, data, len, MSG_DONTWAIT);
		if(ret <= 0) {
			ESP_LOGE(__func__, "send failed (%s)", strerror(errno));
			close(g_vesc_sock);
			g_vesc_sock = 0;
		}
	}
#endif
}

static double read_odometer() {
	double odo = 0;
	size_t length = sizeof(odo);
	nvs_get_blob(g_nvs_handle, "odometer", (void*)&odo, &length);
	if(!isnormal(odo)) {
		return 0;
	} else {
		return odo;
	}
}

esp_err_t store_odometer(double val) {
	return nvs_set_blob(g_nvs_handle, "odometer", (void*)&val, sizeof(val));
}


#define BRAKE_RAMP_UP_MS 200
#define THROTTLE_RAMP_TIME_MS 200

uint8_t motor_disarm() {
	uint8_t tmp = g_motor_armed;
	g_motor_armed = 0;
	return tmp;
}

void motor_arm() {
	g_motor_armed = 1;
}

void motor_set_current_brake_rel(float current_rel) {
	if(have_mcconf) {
		current_rel = clip(current_rel, 0, 1);
		float min_current = fabsf(mcconf.l_current_min);
		float current = current_rel * min_current;
		uint8_t buffer[5];
		buffer[0] = COMM_SET_CURRENT_BRAKE;
		int32_t index = 1;
		buffer_append_int32(buffer, current*1000.0f, &index);
		packet_send_packet(buffer, index, PKT_VESC);
	}
}

void motor_set_current(float current) {
	if(have_mcconf) {
		uint8_t buffer[5];
		buffer[0] = COMM_SET_CURRENT;
		int32_t index = 1;
		buffer_append_float32(buffer, current, 1e5, &index);
		packet_send_packet(buffer, index, PKT_VESC);
	}
}

void motor_set_current_rel(float current_rel) {
	if(have_mcconf) {
		uint8_t buffer[5];
		buffer[0] = COMM_SET_CURRENT_REL;
		int32_t index = 1;
		buffer_append_float32(buffer, current_rel, 1e5, &index);
		packet_send_packet(buffer, index, PKT_VESC);
	}
}

void update_motor_control() {
	if(have_mcconf) {
		uint16_t adc = get_throttle_adc();

		//pressing brakes immediately disables cruise control
		if(g_cc_enabled && adc < ADC_BRAKE_THRESHOLD && !g_brake_controls_disabled) {
			g_cc_enabled = false;
		}

		if(g_cc_enabled) {
			display_set_cruise_icon(1);
			motor_set_current(g_cc_current_set);
		} else {
			static float brake_ramp;
			static float throttle_ramp;
			static uint32_t last_update;
			uint32_t dt = utils_get_dt(&last_update);

			display_set_cruise_icon(0);

			if(!g_motor_armed) {
				motor_set_current_rel(0);
			} else {
				if(adc < 10 && !g_brake_controls_disabled) {
					display_set_brake_icon(1);

					utils_step_towards(&brake_ramp, 1.0, (float)dt / BRAKE_RAMP_UP_MS);

					motor_set_current_brake_rel(brake_ramp);

				} else {
					display_set_brake_icon(0);
					// if brake signal released, release brake immediately
					brake_ramp = 0;
				}

				//brake is released, apply throttle
				if(brake_ramp == 0) {
					if(g_set_power_level < 0) {
						float brkval = utils_map(g_set_power_level, -4, 0, 1, 0);
						motor_set_current_brake_rel(brkval);
					} 
					else if(g_throttle_cal_min != 0) {
						float thrval = utils_map(adc, g_throttle_cal_min, g_throttle_cal_max, 0.0f, 1.0f);
						thrval = clip(thrval, 0.0f, 1.0f);

						utils_step_towards(&throttle_ramp, thrval, (float)dt / THROTTLE_RAMP_TIME_MS);

						if(thrval > 0.02f) {
							motor_set_current_rel(throttle_ramp);
						} else {
							motor_set_current_rel(0);
						}
					}
				}
			}
		}

	}

}

const char* faultToStr(mc_fault_code fault)
{
    switch (fault) {
    case FAULT_CODE_NONE: return NULL;
    case FAULT_CODE_OVER_VOLTAGE: return "FAULT_OVER_VOLTAGE";
    case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_UNDER_VOLTAGE";
    case FAULT_CODE_DRV: return "FAULT_DRV";
    case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_ABS_OVER_CURRENT";
    case FAULT_CODE_OVER_TEMP_FET: return "FAULT_OVER_TEMP_FET";
    case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_OVER_TEMP_MOTOR";
    case FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE: return "FAULT_GATE_DRIVER_OVER_VOLTAGE";
    case FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE: return "FAULT_GATE_DRIVER_UNDER_VOLTAGE";
    case FAULT_CODE_MCU_UNDER_VOLTAGE: return "FAULT_MCU_UNDER_VOLTAGE";
    case FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET: return "FAULT_BOOTING_FROM_WATCHDOG_RESET";
    case FAULT_CODE_ENCODER_SPI: return "FAULT_ENCODER_SPI";
    case FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE: return "FAULT_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE";
    case FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE: return "FAULT_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE";
    case FAULT_CODE_FLASH_CORRUPTION: return "FAULT_FLASH_CORRUPTION";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1: return "FAULT_HIGH_OFFSET_CURRENT_SENSOR_1";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2: return "FAULT_HIGH_OFFSET_CURRENT_SENSOR_2";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3: return "FAULT_HIGH_OFFSET_CURRENT_SENSOR_3";
    case FAULT_CODE_UNBALANCED_CURRENTS: return "FAULT_UNBALANCED_CURRENTS";
    default: return "Unknown fault";
    }
}

//#define DBG_PRINT_VAL(val) ESP_LOGI(#val, "= %f", (float)val)
#define DBG_PRINT_VAL(val)



static void vesc_process_packet_cb(unsigned char *data, unsigned int len) {
	//ESP_LOGI(__func__, "len:%d", len);

	//whole packet received from uart, forward to network (if connected)
	packet_send_packet(data, len, PKT_TCP);
	packet_send_packet(data, len, PKT_UDP);

	//printf("b\n");
	switch(data[0]) {
	case COMM_GET_MCCONF:
	{
		if(!confgenerator_deserialize_mcconf(data+1, &mcconf)) {
			ESP_LOGE(__func__, "Failed to deserialize mcconf, wrong signature?");
			display_show_message("Incompatible vesc version");
		} else {
			have_mcconf = true;

			ESP_LOGI(__func__, "mc_configuration size %d", sizeof(mc_configuration));
			ESP_LOGI(__func__, "Battery Ah %f, cells: %d", 	mcconf.si_battery_ah, mcconf.si_battery_cells);
			ESP_LOGI(__func__, "Motor Poles %d", mcconf.si_motor_poles);
			ESP_LOGI(__func__, "Mot Current Min %f Max %f", mcconf.l_current_min, mcconf.l_current_max);
			ESP_LOGI(__func__, "Wheel diam %f", mcconf.si_wheel_diameter);

			if(mcconf.si_battery_type == BATTERY_TYPE_LIION_3_0__4_2) {
				g_max_watts = 4.2f * mcconf.si_battery_cells * mcconf.l_in_current_max;
				ESP_LOGI(__func__, "Detected max power %d W", (int)g_max_watts);
			}

			display_set_mot_current_minmax(mcconf.l_current_min, mcconf.l_current_max);
			display_set_bat_limits(mcconf.l_battery_cut_start, mcconf.l_battery_cut_end);

			vesc_update_power_level();

			display_set_connected(1);
			motor_arm();

			tach_to_km = (((1.0f * mcconf.si_gear_ratio) / (mcconf.si_motor_poles * 3.0f)) * (mcconf.si_wheel_diameter * M_PI)) / 1000.0f;
        	rpm_to_kmh =  1.0f / (((mcconf.si_motor_poles / 2.0f) * 60.0f *	mcconf.si_gear_ratio) / (mcconf.si_wheel_diameter * M_PI) / 3.6f);
			
		}

		break;
	}
	case COMM_GET_VALUES:
	case COMM_GET_VALUES_SELECTIVE:
	{
        uint32_t mask = 0xFFFFFFFF;
        int32_t idx = 1;
        if (data[0] == COMM_GET_VALUES_SELECTIVE) {
            mask = buffer_get_uint32(data, &idx);
        }

        if (mask & ((1) << 0)) {
        	mc_data.temp_mos = buffer_get_float16(data, 1e1, &idx);
        	DBG_PRINT_VAL(mc_data.temp_mos);
        }
        if (mask & ((1) << 1)) {
        	mc_data.temp_motor = buffer_get_float16(data,1e1, &idx);
        	DBG_PRINT_VAL(mc_data.temp_motor);

        }
        if (mask & ((1) << 2)) {
        	mc_data.current_motor = buffer_get_float32(data,1e2, &idx);
        	DBG_PRINT_VAL(mc_data.current_motor);
        }
        if (mask & ((1) << 3)) {
        	mc_data.current_in = buffer_get_float32(data,1e2, &idx);
        	DBG_PRINT_VAL(mc_data.current_in);
        }
        if (mask & ((1) << 4)) {
        	mc_data.id = buffer_get_float32(data,1e2, &idx);
        	DBG_PRINT_VAL(mc_data.id);
        }
        if (mask & ((1) << 5)) {
        	mc_data.iq = buffer_get_float32(data,1e2, &idx);
        	DBG_PRINT_VAL(mc_data.iq);
        }
        if (mask & ((1) << 6)) {
        	mc_data.duty_now = buffer_get_float16(data,1e3, &idx);
        	DBG_PRINT_VAL(mc_data.duty_now);
        }
        if (mask & ((1) << 7)) {
        	mc_data.rpm = buffer_get_float32(data,1e0, &idx);
        	DBG_PRINT_VAL(mc_data.rpm);
        }
        if (mask & ((1) << 8)) {
        	mc_data.v_in = buffer_get_float16(data, 1e1, &idx);
        	DBG_PRINT_VAL(mc_data.v_in);
        }
        if (mask & ((1) << 9)) {
        	mc_data.amp_hours = buffer_get_float32(data, 1e4, &idx);
        	DBG_PRINT_VAL(mc_data.amp_hours);
        }
        if (mask & ((1) << 10)) {
        	mc_data.amp_hours_charged = buffer_get_float32(data, 1e4, &idx);
        	DBG_PRINT_VAL(mc_data.amp_hours_charged);
        }
        if (mask & ((1) << 11)) {
        	mc_data.watt_hours = buffer_get_float32(data, 1e4, &idx);
        	DBG_PRINT_VAL(mc_data.watt_hours);
        }
        if (mask & ((1) << 12)) {
        	mc_data.watt_hours_charged = buffer_get_float32(data, 1e4, &idx);
        	DBG_PRINT_VAL(mc_data.watt_hours_charged);
        }
        if (mask & ((1) << 13)) {
			if(mc_data.tachometer == 0) {
				g_initial_tach = buffer_get_int32(data, &idx);
				mc_data.tachometer = g_initial_tach;
			} else {
        		mc_data.tachometer = buffer_get_int32(data, &idx);
			}
        	DBG_PRINT_VAL(mc_data.tachometer);
        }
        if (mask & ((1) << 14)) {
        	mc_data.tachometer_abs = buffer_get_int32(data, &idx);
        	DBG_PRINT_VAL(mc_data.tachometer_abs);
        }
        if (mask & ((1) << 15)) {
        	mc_data.fault_code = data[idx++];
        	mc_data.fault_str = faultToStr(mc_data.fault_code);
        	if(mc_data.fault_str) {
        		display_show_message(mc_data.fault_str);
        	}
        	DBG_PRINT_VAL(mc_data.fault_code);
        }

        if (len - idx >= 4) {
            if (mask & ((1) << 16)) {
            	mc_data.position = buffer_get_float32(data, 1e6, &idx);
            	DBG_PRINT_VAL(mc_data.position);
            }
        } else {
        	mc_data.position = -1.0;
        }

        if (len - idx >= 1) {
            if (mask & ((1) << 17)) {
            	mc_data.vesc_id = data[idx++];
            	DBG_PRINT_VAL(mc_data.vesc_id);
            }
        } else {
        	mc_data.vesc_id = 255;
        }

        if (len - idx >= 6) {
            if (mask & ((1) << 18)) {
            	mc_data.temp_mos_1 = buffer_get_float16(data, 1e1, &idx);
            	mc_data.temp_mos_2 = buffer_get_float16(data, 1e1, &idx);
            	mc_data.temp_mos_3 = buffer_get_float16(data, 1e1, &idx);
            	DBG_PRINT_VAL(mc_data.temp_mos_1);
            	DBG_PRINT_VAL(mc_data.temp_mos_2);
            	DBG_PRINT_VAL(mc_data.temp_mos_3);
            }
        }

        if (len - idx >= 8) {
            if (mask & ((1) << 19)) {
            	mc_data.vd = buffer_get_float32(data, 1e3, &idx);
            	DBG_PRINT_VAL(mc_data.vd);
            }
            if (mask & ((1) << 20)) {
            	mc_data.vq = buffer_get_float32(data, 1e3, &idx);
            	DBG_PRINT_VAL(mc_data.vq);
            }
        }

		static uint32_t prev_update;
		uint32_t dt = utils_get_dt(&prev_update);

		static float current_motor_filt;
		static float duty_now_filt;
		static float v_in_filt;
		static float curr_in_filt;
		static float power_filt;

		float alpha = (float)dt / (dt + 100); // 100ms filter

		UTILS_LP_FAST(current_motor_filt, mc_data.current_motor, alpha);
		UTILS_LP_FAST(duty_now_filt, mc_data.duty_now, alpha);
		UTILS_LP_FAST(v_in_filt, mc_data.v_in, alpha);
		UTILS_LP_FAST(curr_in_filt, mc_data.current_in, alpha);
		UTILS_LP_FAST(power_filt, mc_data.current_in * mc_data.v_in, alpha);

        display_set_duty(duty_now_filt);
        display_set_mos_temp(mc_data.temp_mos);
        display_set_curr_power(power_filt);
        display_set_bat_info(v_in_filt, curr_in_filt);
		
        display_set_mot_current(current_motor_filt);
        display_set_energy(mc_data.amp_hours_charged, mc_data.amp_hours);


        float trip_km = mc_data.tachometer * tach_to_km;
        display_set_trip(trip_km);

        static uint32_t trip_time_ms;
        static uint32_t prev_trip_timestamp;

        display_set_mos_temp(mc_data.temp_mos);

        if(abs(mc_data.rpm) > 50) {
        	trip_time_ms += (platform_time_ms() - prev_trip_timestamp);

        	display_set_triptime(trip_time_ms);

        	float trip_km = (mc_data.tachometer - g_initial_tach) * tach_to_km;
        	//ESP_LOGI(__func__, "trip rel %f %f", trip_km, (((float)trip_time_ms/1000.0f)/3600.0f));
			if(trip_time_ms != 0)
        		display_set_avgspeed(trip_km / (((float)trip_time_ms/1000.0f)/3600.0f));
        }

        prev_trip_timestamp = platform_time_ms();
		if(rpm_to_kmh != 0) {
			float kmh = mc_data.rpm * rpm_to_kmh;
			static float speed_filt;
			UTILS_LP_FAST(speed_filt, kmh, alpha);
        	display_set_speed(speed_filt);
			if(speed_filt < 10) {
				g_cc_enabled = false;
			}
		}


        if(prev_odo == 0) {
        	prev_odo = mc_data.tachometer;
        } else {
        	if( mcconf.si_gear_ratio != 0 && mcconf.si_motor_poles != 0) {
        		int diff = (mc_data.tachometer - prev_odo);
        		if(diff > 0) {
        			stored_odo += (double)diff * tach_to_km;
		        	prev_odo = mc_data.tachometer;
        		}
        	}
        }
        display_set_odo(stored_odo);

        if(platform_time_ms() - t_odo_save > 5000 && fabsf(read_odometer()-stored_odo) > 0.05f) {
        	ESP_LOGI(__func__, "Store odo value %f", stored_odo);

        	esp_err_t ret = store_odometer(stored_odo);
        	if(ret != ESP_OK) {
        		display_show_message("ODO store fail");
        	}
        	t_odo_save = platform_time_ms();
        }

		//if(!g_tcp_client_sock) {
			//uint8_t req = COMM_GET_VALUES;
			//packet_send_packet(&req, 1, PKT_VESC);
			update_motor_control();
		//}
	}
	}
}



//PUBLIC
uint16_t get_throttle_adc() {
	static uint16_t samples[5];
	static uint8_t sample_idx;
	uint16_t adc_data = 0;
	ESP_ERROR_CHECK(adc_read(&adc_data));

	samples[sample_idx++] = adc_data;
	if(sample_idx >= 5) sample_idx = 0;

	return utils_median_5_int(samples[0], samples[1], samples[2], samples[3], samples[4]);
}

//PUBLIC
void set_throttle_calibration(uint16_t min, uint16_t max) {
	if(min <= max) {
		if(max < 1024)
			g_throttle_cal_max = max;
		if(min < 1024)
			g_throttle_cal_min = min;
	}

	if(abs(max-min) > 100) {
		esp_err_t ret;
		ret = nvs_set_u16(g_nvs_handle, "thrcal_min", min);
		if(ret != ESP_OK) {
			ESP_LOGE(__func__, "failed to store cal (%s)", esp_err_to_name(ret));
		}
		nvs_set_u16(g_nvs_handle, "thrcal_max", max);

	}
}

void vesc_poll_data() {
	display_set_connected(0);

	uint8_t req = COMM_GET_MCCONF;
	packet_send_packet(&req, 1, 0);

	req = COMM_GET_VALUES;
	packet_send_packet(&req, 1, 0);


	update_motor_control();
	//printf("heap %d\n", esp_get_free_heap_size());
}

#ifdef CONFIG_ESP_VESC_UART
void vesc_uart_task(void* param) {
	uart_event_t event;
	uint8_t buffer[128];
	unsigned long prevWake = 0;
	while(1) {
		int res;
		while((res = uart_read_bytes(0, buffer, sizeof(buffer), 1 /* tick */)) > 0) {
			for(int i = 0; i < res; i++) {
				packet_process_byte(buffer[i], PKT_VESC);
			}
		}

		vTaskDelayUntil(&prevWake, 20/portTICK_PERIOD_MS);

		if(!have_mcconf) {
			vTaskDelayMs(100);
			vesc_poll_data();
		} else {
			uint8_t req = COMM_GET_VALUES;
			packet_send_packet(&req, 1, PKT_VESC);
		}
	}
}
#endif

#ifndef CONFIG_ESP_VESC_UART
void vesc_net_task(void* param) {
	int ret;


	while(1) {
		if(!g_vesc_sock) {
			g_vesc_sock = socket(AF_INET, SOCK_DGRAM, 0);

#ifdef TCP
			int one = 1;
			setsockopt(g_vesc_sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

			struct linger sl;
			sl.l_onoff = 0;		/* non-zero value enables linger option in kernel */
			sl.l_linger = 0;	/* timeout interval in seconds */
			setsockopt(g_vesc_sock, SOL_SOCKET, SO_LINGER, &sl, sizeof(sl));

			struct timeval tv;
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			setsockopt(g_vesc_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
#endif

			struct sockaddr_in addr;
			addr.sin_addr.s_addr = inet_addr("192.168.4.1");
			addr.sin_port = htons(2323);
			addr.sin_family = AF_INET;
			ret = connect(g_vesc_sock, (struct sockaddr*)&addr, sizeof(addr));
			if(ret < 0) {
				ESP_LOGE(__func__, "connect failed (%s)", strerror(errno));
				close(g_vesc_sock);
				g_vesc_sock = 0;
				vTaskDelay(200 / portTICK_PERIOD_MS);
				continue;
			}
			ESP_LOGI(__func__, "connected");

			vesc_poll_data();
			;
		}

		if(!g_vesc_sock) {
			vTaskDelay(100 / portTICK_PERIOD_MS);
			continue;
		}

		fd_set fds;
		struct timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;

		FD_ZERO(&fds);
		FD_SET(g_vesc_sock, &fds);

		if((ret = select(g_vesc_sock+1, &fds, NULL, NULL, &tv) > 0)) {
			char buf[100];
			ret = recv(g_vesc_sock, buf, sizeof(buf), MSG_DONTWAIT);
			if(ret > 0) {
				for(int i = 0; i < ret; i++) {
					packet_process_byte(buf[i], PKT_VESC);
				}
			} else {
				ESP_LOGE(__func__, "recv() failed");
				close(g_vesc_sock);
				g_vesc_sock = 0;
			}
		} else if(ret < 0) {
			ESP_LOGE(__func__, "select() failed");
			close(g_vesc_sock);
			g_vesc_sock = 0;
		} else {
			/* timeout, hmm let's request data again */
			ESP_LOGI(__func__, "vesc_poll_data");
			vesc_poll_data();
		}

	}
}
#endif

#ifdef CONFIG_ESP_VESC_STA
void wifi_init_sta()
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );

    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(__func__, "wifi_init_sta finished.");

}
#endif

void on_cruise_control_request(uint8_t long_press) {
	if(mc_data.rpm * rpm_to_kmh < 5 && long_press) {
		display_show_menu();
	} else
	if(g_cc_enabled) {
		g_cc_enabled = false;
	} else
	if(mc_data.rpm * rpm_to_kmh > 10 && long_press && mc_data.current_motor > 0) {
		g_cc_current_set = mc_data.current_motor;
		g_cc_rpm_set = mc_data.rpm;
		ESP_LOGI(__func__, "set cruise %f", g_cc_current_set);
		g_cc_enabled = true;
	}
}

void on_power_limit_request(int8_t plimit) {
	if(have_mcconf && plimit >= 0) {
		vesc_update_power_level();
	}
}


void app_main(void) {
#if CONFIG_GPIO_TFT_LED >= 0
	gpio_set_direction(CONFIG_GPIO_TFT_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_GPIO_TFT_LED, 0);
#endif

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

#ifdef CONFIG_ESP_VESC_UART
	esp_log_set_putchar(putc_remote);
#endif
	//esp_log_set_putchar(putc_noop);

	/* we are not using uart rx, disable interrupt */
	//uart_disable_rx_intr(0);
	xTaskCreate(&dbg_task, "dbg_main", 800, NULL, 4, NULL);


#ifdef CONFIG_ESP_VESC_UART

	uart_driver_install(0, 512, 256, 16, &uart_events_queue, 0);
	uart_set_baudrate(0, CONFIG_ESP_UART_BAUD);
#else
	//disable uart interrupts
	uart_intr_config_t conf = {};
	conf.intr_enable_mask = 0;
	uart_intr_config(0, &conf);
#endif

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(nvs_open("config", NVS_READWRITE, &g_nvs_handle));

	vTaskPrioritySet(NULL, 3);

#ifdef CONFIG_ESP_VESC_STA
	wifi_init_sta();
#endif

#ifdef CONFIG_ESP_AP_MODE
	wifi_init_softap();
#endif

	adc_config_t adc_config;
	adc_config.mode = ADC_READ_TOUT_MODE;
	adc_config.clk_div = 8;
	ESP_ERROR_CHECK(adc_init(&adc_config));



	httpd_start();

	esp_wifi_set_ps(WIFI_PS_NONE);

	ESP_LOGI(__func__, "display_setup begin\n");
	display_setup();
	ESP_LOGI(__func__, "display_setup end\n");

#if CONFIG_GPIO_TFT_LED >= 0
	gpio_set_level(CONFIG_GPIO_TFT_LED, 1);
#endif

	packet_init(vesc_packet_send_cb, vesc_process_packet_cb, PKT_VESC);
	packet_init(_net_pkt_udp_snd_handler, _net_pkt_handler, PKT_UDP);
	packet_init(_net_pkt_tcp_snd_handler, _net_pkt_handler, PKT_TCP);

	stored_odo = read_odometer();
	display_set_odo(stored_odo);

	ret = nvs_get_u16(g_nvs_handle, "thrcal_min", &g_throttle_cal_min);
	if(ret != ESP_OK) {
		ESP_LOGE(__func__, "failed to load throttle cal");
	}
	nvs_get_u16(g_nvs_handle, "thrcal_max", &g_throttle_cal_max);

	if(g_throttle_cal_min == 0 || g_throttle_cal_max == 0 || 
			g_throttle_cal_min > g_throttle_cal_max ||
			g_throttle_cal_max > 1023 || g_throttle_cal_min > 1023) {
		g_throttle_cal_min = 0;
		g_throttle_cal_max = 0;

		display_show_message("Throttle not calibrated");
	}

	display_set_cruise_control_cb(on_cruise_control_request);
	display_set_power_level_cb(on_power_limit_request);

#ifdef CONFIG_ESP_VESC_UART
	xTaskCreate(&vesc_uart_task, "vesc_uart", 1500, NULL, 3, NULL);
#else
	xTaskCreate(&vesc_net_task, "vesc_net", 1500, NULL, 3, NULL);
#endif

	xTaskCreate(&net_proxy_task, "net_proxy_task", 1500, NULL, 3, NULL);

	//ota_tftp_init_server(69, 7);

	extern void netcat_ota_task(void* port);
	xTaskCreate(&netcat_ota_task, "netcat-ota", 1500, (void*)120, 7, NULL);

	ESP_LOGI(__func__, "Free heap %d\n", esp_get_free_heap_size());
	ESP_LOGI(__func__, "vdd33 %d", esp_wifi_get_vdd33());

	display_run();
}

