#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <driver/gpio.h>
#include <driver/spi.h>
#include "esp8266/gpio_struct.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <lvgl.h>
#include "ILI9341.h"
#include "ILI9225.h"
#include "spi.h"
#include "display.h"
#include <math.h>
#include "esp_system.h"
#include "utils.h"

#include <string>

LV_IMG_DECLARE(brake);
LV_IMG_DECLARE(cruise_control_img);

#ifdef CONFIG_LCD_TOUCH
#include "XPT2046.h"

static XPT2046 touch;

lv_indev_drv_t indev_drv;
lv_indev_t * touch_indev;

#endif

lv_indev_drv_t btn_indev_drv;
lv_indev_t* btn_indev;

static IRAM_ATTR lv_color_t lv_buf_1[LV_HOR_RES_MAX * 10];

static lv_disp_t* disp;

static lv_group_t* ctrl_group;

static lv_obj_t *bar_power;
static lv_obj_t* lbl_speed;
static lv_obj_t* lbl_odo;
static lv_obj_t* lbl_trip;
static lv_obj_t* lbl_trip_val;
static lv_obj_t* lbl_func;
//static lv_obj_t* lbl_bat;
static lv_obj_t* lbl_pow;
static lv_obj_t* lbl_mah;
static lv_obj_t* lbl_mot_curr;
static lv_obj_t *linemeter;
static lv_obj_t* duty_gauge;
static lv_theme_t * theme;
static lv_obj_t* lbl_volts;
static lv_obj_t* lbl_amps;
static lv_obj_t* lbl_wifisymbol;
static lv_obj_t* lbl_message;
static lv_obj_t *power_label;

static void(*cruise_control_func)(uint8_t long_press);
static void(*power_lvl_changed_fn)(int8_t level);

static bool night_mode = 0;

static lv_obj_t* brake_icon ;
static lv_obj_t* cruise_control_icon;

int8_t g_set_power_level = 0;

static bool displ_message_updated;
static std::string displ_message_curr;

enum {
	DISPL_TRIP,
	DISPL_TRIPTIME,
	DISPL_AVS,
	DISPL_FET,
	DISPL_WHKM,
	DISPL_LAST
};

static uint8_t g_cur_display = DISPL_TRIP;

static float g_triptime;
static float g_avgspeed;
static float g_trip;
static float g_mos_temp;
static float g_wh_km;

static float bat_cut_start;
static float bat_cut_end;
int ctr;

static uint32_t g_tm_prev_change;

//#define DOUBLE_BUFFER

#ifdef DOUBLE_BUFFER
struct flush_data {
	lv_disp_drv_t * disp;
	lv_area_t area;
	lv_color_t * color_p;
};
static IRAM_ATTR lv_color_t lv_buf_2[LV_HOR_RES_MAX * 10];
static xQueueHandle flush_q;
static TaskHandle_t flush_notify_task_handle;

#endif

static SemaphoreHandle_t lvgl_mutex;
static lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
static lv_disp_buf_t disp_buf;

#define LVGL_LOCK()   //xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY );
#define LVGL_UNLOCK() //xSemaphoreGiveRecursive(lvgl_mutex );

#define ON_CHANGED(x, y) { static typeof(x) _value; typeof(x) value = x; if(_value != value) { _value = value; y }}

#define ON_CHANGED_2(x1, x2, y) static typeof(x1) _value_1; \
								static typeof(x2) _value_2;	\
								typeof(x1) value_1 = x1;\
								typeof(x2) value_2 = x2;\
								if(value_1 != _value_1 || value_2 != _value_2) { _value_1 = value_1; _value_2 = value_2; y }


#ifdef DOUBLE_BUFFER
void IRAM_ATTR lvgl_flush_task(void* parm) {
	struct flush_data args;
	ESP_LOGI(__func__, "start");

	while(1) {
		xQueueReceive(flush_q, &args, portMAX_DELAY);
		//ESP_LOGI(__func__, "flush");
		//printf("win2 x1:%d y1:%d x2:%d y2:%d %p\n", args.area.x1, args.area->y1, args.area->x2, args.area->y2, args.color_p);
		uint32_t start = esp_get_time();
#ifdef CONFIG_ESP_TFT_ILI9341
		ILI9341_setWindow(args.area.x1, args.area.y1, args.area.x2, args.area.y2);
#elif CONFIG_ESP_TFT_ILI9225
		ILI9225_setWindow(args.area.x1, args.area.y1, args.area.x2, args.area.y2);
#endif

		spi_beginTransaction(CONFIG_GPIO_TFT_CS);

		int count = ((args.area.y2-args.area.y1+1)*(args.area.x2-args.area.x1+1));
		spi_send_aligned(args.color_p, count*sizeof(lv_color_t));

		spi_endTransaction();

		lv_disp_flush_ready(args.disp);         /* Indicate you are ready with the flushing*/
		//printf("flush %dpx %d us\n", count, esp_get_time()-start);
		if(flush_notify_task_handle) {
			xTaskNotifyGive(flush_notify_task_handle);
			flush_notify_task_handle = 0;
		}
	}
}
#endif

static IRAM_ATTR void lvgl_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p) {
#ifdef DOUBLE_BUFFER
	struct flush_data args;
	args.disp = disp;
	args.area = *area;
	args.color_p = color_p;

	xQueueSend(flush_q, &args, portMAX_DELAY);
#else
#ifdef CONFIG_ESP_TFT_ILI9341
	//printf("flush %d %d %d %d\n", area->x1, area->y1, area->x2, area->y2);
	ILI9341_setWindow(area->x1, area->y1, area->x2, area->y2);
#elif CONFIG_ESP_TFT_ILI9225
	ILI9225_setWindow(area->x1, area->y1, area->x2, area->y2);
#endif

	spi_beginTransaction(CONFIG_GPIO_TFT_CS);
	int count = ((area->y2-area->y1+1)*(area->x2-area->x1+1));
	//printf("send count:%d %x\n", count, uint32_t(color_p));

	spi_send_aligned(color_p, count*sizeof(lv_color_t));

	spi_endTransaction();

    lv_disp_flush_ready(disp);         /* Indicate you are ready with the flushing*/
#endif
}

#ifdef CONFIG_LCD_TOUCH
bool touch_read_cb(lv_indev_drv_t * drv, lv_indev_data_t* data)
{
	static uint16_t last_x, last_y;
	uint16_t x, y;
	touch.getPosition(x,y, touch.MODE_SER, 16);
	if(x == 0xffff) {
    	data->state = LV_INDEV_STATE_REL;
	} else {
		last_x = x;
		last_y = y;
		data->state = LV_INDEV_STATE_PR;

		uint16_t i,j;
		touch.getRaw(i,j);
		printf("%d %d %d %d\n", last_x, last_y, i, j);
	}
    data->point.x = last_x;
    data->point.y = last_y;
    return false; /*No buffering now so no more data read*/
}
#endif

void monitor_cb(struct _disp_drv_t * disp_drv, uint32_t time, uint32_t px) {
	ESP_LOGI(__func__, "refreshed %dpx in %dms", px, time);
}

extern "C"
void vApplicationTickHook() {
	lv_tick_inc(portTICK_PERIOD_MS);
}


void make_power_label(char* buf, int level) {
	int pos = 0;

	for(int i = -4; i <= 4; i++) {
		if(i < 0) {
			if(i < level ) {
				const char str[] = "#EECCCC " LV_SYMBOL_STOP "#";
				strcpy(buf, str);
				buf+= strlen(str);
			} else{ 
				const char str[] =  "#EE1010 " LV_SYMBOL_STOP "#";
				strcpy(buf, str);
				buf+= strlen(str);
			}
			
		} else if(i > 0) {
			
			if(i > level && i != 0) {
				const char str[] = "#CCEECC " LV_SYMBOL_STOP "#";
				strcpy(buf, str);
				buf+= strlen(str);
			} else {
				const char str[] = "#10EE10 " LV_SYMBOL_STOP "#";
				strcpy(buf, str);
				buf+= strlen(str);
			}
		}
		if(i == 0) {
			*buf++ = ' ';
			//*buf++ = '|';
			*buf++ = ' ';
		}
	}
}

extern "C" void motor_set_current_rel(float current_rel);
extern "C" void motor_set_current_brake_rel(float current_rel);

extern "C"
void display_setup() {
	lvgl_mutex = xSemaphoreCreateRecursiveMutex();
#ifdef DOUBLE_BUFFER
	flush_q = xQueueCreate(2, sizeof(struct flush_data));
#endif
	spidrv_init();

#ifdef CONFIG_LCD_TOUCH
	touch.begin(240,320);
	touch.setRotation((XPT2046::rotation_t)3);
	touch.setCalibration(411, 243, 3866, 3795);
#endif

#ifdef CONFIG_ESP_TFT_ILI9341
	ILI9341_init();
#elif CONFIG_ESP_TFT_ILI9225
	ILI9225_init();
#else
#error "Unknown LCD"
#endif

    lv_init();
	lv_log_register_print_cb([](lv_log_level_t level, const char *, uint32_t, const char *, const char * desc) {
		switch(level) {
			case LV_LOG_LEVEL_WARN:
				ESP_LOGW("LVGL", desc);
				break;
			case LV_LOG_LEVEL_INFO:
				ESP_LOGI("LVGL", desc);
				break;
			case LV_LOG_LEVEL_ERROR:
				ESP_LOGE("LVGL", desc);
				break;
			default:
				ESP_LOGI("LVGL", desc);

		}
	});


#ifdef DOUBLE_BUFFER
	lv_disp_buf_init(&disp_buf, lv_buf_1, lv_buf_2, sizeof(lv_buf_1)/sizeof(lv_buf_1[0]));    /*Initialize the display buffer*/
#else
    lv_disp_buf_init(&disp_buf, lv_buf_1, 0, sizeof(lv_buf_1)/sizeof(lv_buf_1[0]));    /*Initialize the display buffer*/
#endif
    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.flush_cb = lvgl_flush;    /*Set your driver function*/

    //disp_drv.monitor_cb = monitor_cb;
    disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
    disp = lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

#ifdef CONFIG_LCD_TOUCH
	lv_indev_drv_init(&indev_drv);      
	indev_drv.type = LV_INDEV_TYPE_POINTER;           
	indev_drv.read_cb =  touch_read_cb; 
	/*Register the driver in LittlevGL and save the created input device object*/
	touch_indev = lv_indev_drv_register(&indev_drv);
#endif

#if CONFIG_GPIO_BTN_UP>=0 && CONFIG_GPIO_BTN_DN>=0 && CONFIG_GPIO_BTN_ENT>=0
	gpio_config_t cfg = {};
	cfg.mode = GPIO_MODE_INPUT;
	cfg.pin_bit_mask = BIT(CONFIG_GPIO_BTN_UP) | BIT(CONFIG_GPIO_BTN_DN) | BIT(CONFIG_GPIO_BTN_ENT);
	cfg.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&cfg);

	lv_indev_drv_init(&btn_indev_drv);      
	btn_indev_drv.type = LV_INDEV_TYPE_ENCODER;           
	btn_indev_drv.read_cb =  [](lv_indev_drv_t * drv, lv_indev_data_t* data) {
		static bool prev_states[3];
		static uint32_t time_pressed;

		if(gpio_get_level((gpio_num_t)CONFIG_GPIO_BTN_UP) != prev_states[0]) {
			//ESP_LOGI(__func__, "up");
			prev_states[0] = gpio_get_level((gpio_num_t)CONFIG_GPIO_BTN_UP);

			if(prev_states[0] == 0) {
				data->enc_diff = -1;
				if(ctrl_group->frozen) {
					display_set_power_level(g_set_power_level+1);
				}
			}
		}

		if(gpio_get_level((gpio_num_t)CONFIG_GPIO_BTN_DN) != prev_states[1]) {
			//ESP_LOGI(__func__, "dn");
			prev_states[1] = gpio_get_level((gpio_num_t)CONFIG_GPIO_BTN_DN);
			
			if(prev_states[1] == 0) {
				data->enc_diff = 1;
				if(ctrl_group->frozen) {
					display_set_power_level(g_set_power_level-1);
				}
			}
		}

		if(gpio_get_level((gpio_num_t)CONFIG_GPIO_BTN_ENT) == 0 ) {
			data->state = LV_BTN_STATE_PRESSED;
			if(ctrl_group->frozen) {
				if(time_pressed == 0) {
					time_pressed = lv_tick_get();
					if(cruise_control_func)
						cruise_control_func(false);
				}
				if(time_pressed != 1 && lv_tick_get() - time_pressed > 1000) {
#if CONFIG_GPIO_BTN_ENT == 0
					//enter flash mode
					if(prev_states[0] == 0) {
						esp_restart();
					}
#endif
					ESP_LOGI("event", "CC action");
					if(cruise_control_func) 
						cruise_control_func(true);
					time_pressed = 1;
				}
			}

		} else {
			data->state = LV_BTN_STATE_RELEASED;
			time_pressed = 0;
		}

		return false; /*No buffering now so no more data read*/
	}; 

	/*Register the driver in LittlevGL and save the created input device object*/
	btn_indev = lv_indev_drv_register(&btn_indev_drv);

	ctrl_group = lv_group_create();
	lv_group_focus_freeze(ctrl_group, true);
	lv_indev_set_group(btn_indev, ctrl_group);
#endif

	lv_obj_t* screen = lv_obj_create(NULL, NULL);
	lv_scr_load(screen); 

#ifdef CONFIG_ESP_TFT_ILI9341
	// screen = lv_obj_create(screen, 0);
	// lv_obj_set_size(screen, 320, 176);
    // lv_obj_align(screen, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);
	// lv_obj_set_style(screen, theme->style.scr);
#endif


    lbl_speed = lv_label_create(screen, NULL);

    lv_label_set_text(lbl_speed, "00");
	lv_obj_set_auto_realign(lbl_speed, true);
#ifdef CONFIG_ESP_TFT_ILI9341
	lv_obj_set_style_local_text_font(lbl_speed, 0, LV_STATE_DEFAULT, &large_80);
#else
	lv_obj_set_style_local_text_font(lbl_speed, 0, LV_STATE_DEFAULT, &lv_font_montserrat_48);
#endif
    lv_obj_align(lbl_speed, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t* lbl_kmh = lv_label_create(screen, NULL);
    lv_label_set_text(lbl_kmh, "km/h");
    lv_obj_align(lbl_kmh, lbl_speed, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    bar_power = lv_bar_create(screen, NULL);
    lv_obj_set_height(bar_power, 10);
    lv_bar_set_range(bar_power, -100, 100);
    lv_bar_set_value(bar_power, 0, 0);
    lv_bar_set_sym(bar_power, true);
    lv_obj_align(bar_power, lbl_kmh, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    linemeter = lv_linemeter_create(screen, NULL);
    lv_linemeter_set_range(linemeter, 0, 50);
    lv_obj_set_height(linemeter, LV_HOR_RES_MAX*0.5f);
    lv_obj_set_width(linemeter, LV_HOR_RES_MAX*0.5f);
    lv_obj_align(linemeter, lbl_speed, LV_ALIGN_CENTER, 0, 0);

	lv_obj_set_click(linemeter, true);
	lv_obj_set_event_cb(linemeter, [](lv_obj_t * obj, lv_event_t event) {
		if(event == LV_EVENT_LONG_PRESSED) {
			printf("pressed\n");
		}

	});

    
	lv_obj_t* odo_container = lv_cont_create(screen, 0);
	lv_obj_set_style_local_text_font(odo_container, 0, LV_STATE_DEFAULT, &iosevka_20);
	lv_obj_set_style_local_pad_all(odo_container, 0, LV_STATE_DEFAULT, 4);


	lv_cont_set_fit(odo_container, LV_FIT_TIGHT);

	lbl_func = lv_label_create(odo_container, NULL);
	lv_label_set_text(lbl_func, "ODO");
	lv_obj_set_style_local_text_font(lbl_func, 0, LV_STATE_DEFAULT, &lv_font_montserrat_16);

	lv_obj_align(lbl_func, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 2, -2);

	lbl_odo = lv_label_create(odo_container, NULL);

	lv_label_set_text(lbl_odo, "00000");
	lv_obj_align(lbl_odo, lbl_func, LV_ALIGN_OUT_RIGHT_MID, 2, 0);

	lv_cont_set_layout(odo_container, LV_LAYOUT_ROW_MID);
	lv_cont_set_fit2(odo_container, LV_FIT_TIGHT, LV_FIT_TIGHT);
	lv_obj_align(odo_container, 0, LV_ALIGN_IN_BOTTOM_LEFT, 2, -2);
	lv_obj_set_auto_realign(odo_container, 1);
    
    
	lv_obj_t* trp_info_container = lv_cont_create(screen, 0);
	lv_cont_set_fit(trp_info_container, LV_FIT_TIGHT);
	lv_obj_set_style_local_text_font(trp_info_container, 0, LV_STATE_DEFAULT, &iosevka_20);
	lv_obj_set_style_local_pad_all(trp_info_container, 0, LV_STATE_DEFAULT, 4);

	lbl_trip = lv_label_create(trp_info_container, NULL);
	lv_label_set_text(lbl_trip, "TRP");
	lv_obj_set_style_local_text_font(lbl_trip, 0, LV_STATE_DEFAULT, &lv_font_montserrat_16);



	lbl_trip_val = lv_label_create(trp_info_container, NULL);
	lv_label_set_text(lbl_trip_val, "0.00");

	lv_cont_set_layout(trp_info_container, LV_LAYOUT_ROW_MID);
	lv_cont_set_fit2(trp_info_container, LV_FIT_TIGHT, LV_FIT_TIGHT);
	lv_obj_align(trp_info_container, 0, LV_ALIGN_IN_BOTTOM_RIGHT, -2, -2);
	lv_obj_set_auto_realign(trp_info_container, 1);

    
//    lbl_bat = lv_label_create(lv_disp_get_layer_top(disp), NULL);
//    //lv_label_set_style(lbl_trip_val, LV_LABEL_STYLE_MAIN, &font_22_style);
//    lv_label_set_text(lbl_bat, LV_SYMBOL_BATTERY_3 " 44.1V 2.3A");
//    lv_obj_align(lbl_bat, 0, LV_ALIGN_IN_TOP_LEFT, 2, -2);
//    lv_label_set_recolor(lbl_bat, 1);

    lbl_mah = lv_label_create(screen, NULL);
#ifdef CONFIG_ESP_TFT_ILI9341
	lv_obj_set_style_local_text_font(lbl_mah, 0, LV_STATE_DEFAULT, &iosevka_20);
#else
	lv_obj_set_style_local_text_font(lbl_mah, 0, LV_STATE_DEFAULT, &iosevka_14);
#endif
    lv_label_set_text_fmt(lbl_mah, "mAh\nDSG%05d\nCHG%05d", 0, 0);
    lv_label_set_align(lbl_mah, LV_LABEL_ALIGN_RIGHT);

    lv_obj_align(lbl_mah, 0, LV_ALIGN_IN_TOP_RIGHT, -2, 0);

    lv_obj_t* cont = lv_cont_create(screen, NULL);
    lv_obj_set_style_local_pad_all(cont, 0, LV_STATE_DEFAULT, 4);

    //lv_obj_set_style(cont, &cont_style);

    lbl_pow = lv_label_create(cont, NULL);
    lv_label_set_text(lbl_pow, "0W");
#ifdef CONFIG_ESP_TFT_ILI9341
    lv_obj_set_width(cont, LV_HOR_RES_MAX*0.27);
	lv_obj_set_style_local_text_font(lbl_pow, 0, LV_STATE_DEFAULT, &iosevka_20);
#else
    lv_obj_set_width(cont, LV_HOR_RES_MAX*0.27);
	lv_obj_set_style_local_text_font(lbl_pow, 0, LV_STATE_DEFAULT, &iosevka_20);
#endif
	lv_obj_set_style_local_pad_all(lbl_pow, 0, LV_STATE_DEFAULT, 4);
    lv_obj_align(lbl_pow, 0, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_auto_realign(lbl_pow, 1);

    lv_obj_align(cont, 0, LV_ALIGN_IN_LEFT_MID, 2, 0);

    cont = lv_cont_create(screen, NULL);
	lv_obj_set_style_local_pad_all(cont, 0, LV_STATE_DEFAULT, 4);

    lbl_mot_curr = lv_label_create(cont, NULL);
    lv_label_set_text(lbl_mot_curr, "0A");
	lv_obj_set_style_local_text_font(lbl_mot_curr, 0, LV_STATE_DEFAULT, &iosevka_20);
    lv_obj_set_auto_realign(lbl_mot_curr, 1);
    lv_obj_set_width(cont, LV_HOR_RES_MAX*0.27);
    lv_obj_align(lbl_mot_curr, 0, LV_ALIGN_CENTER, 0, 0);

    lv_obj_align(cont, 0, LV_ALIGN_IN_RIGHT_MID, -2, 0);

    /* status bar */
    lv_obj_t* statusbar = lv_cont_create(screen, NULL);
	lv_obj_set_style_local_bg_opa(statusbar, 0, LV_STATE_DEFAULT, 0);
	lv_obj_set_style_local_pad_all(statusbar, 0, LV_STATE_DEFAULT, 1);
#ifdef CONFIG_ESP_TFT_ILI9341
	lv_obj_set_style_local_text_font(statusbar, 0, LV_STATE_DEFAULT, &lv_font_montserrat_22);
#else
	lv_obj_set_style_local_text_font(statusbar, 0, LV_STATE_DEFAULT, &lv_font_montserrat_16);
#endif
    lbl_wifisymbol = lv_label_create(statusbar, NULL);
    lv_label_set_text(lbl_wifisymbol, LV_SYMBOL_WIFI);
	lv_obj_set_style_local_opa_scale(lbl_wifisymbol, 0, LV_STATE_DEFAULT, 64);
	

    lv_obj_t* lbl_battsymbol = lv_label_create(statusbar, NULL);
    lv_label_set_text(lbl_battsymbol, LV_SYMBOL_BATTERY_3);

    lbl_volts = lv_label_create(statusbar, NULL);
    lv_label_set_text(lbl_volts, "0.0V");
    lv_label_set_recolor(lbl_volts, 1);

    lbl_amps = lv_label_create(statusbar, NULL);
    lv_label_set_text(lbl_amps, "0.0A");

    lv_cont_set_layout(statusbar, LV_LAYOUT_ROW_MID);
    lv_cont_set_fit(statusbar, LV_FIT_TIGHT);

    lv_obj_align(statusbar, 0, LV_ALIGN_IN_TOP_LEFT, 2, -2);

    /* duty gauge */
    duty_gauge = lv_gauge_create(screen, NULL);
    lv_gauge_set_range(duty_gauge, 0, 100);
    lv_obj_set_width(duty_gauge, LV_HOR_RES_MAX*0.32);
    lv_obj_set_height(duty_gauge, LV_HOR_RES_MAX*0.32);
    lv_gauge_set_scale(duty_gauge, 180, 10, 0);
    lv_gauge_set_value(duty_gauge, 0, 0);
    lv_obj_align(duty_gauge, 0, LV_ALIGN_IN_LEFT_MID, 2, -34);
    /* ********** */
	auto duty_lbl = lv_label_create(screen, NULL);
	lv_label_set_text(duty_lbl, "Duty%");
	lv_obj_set_style_local_text_font(duty_lbl, 0, LV_STATE_DEFAULT, &lv_font_unscii_8);
	lv_obj_align(duty_lbl, duty_gauge, LV_ALIGN_CENTER, 0, 14);


	lbl_message = lv_label_create(screen, NULL);
	lv_label_set_recolor(lbl_message, true);
	lv_label_set_static_text(lbl_message, "");
	lv_obj_set_style_local_text_color(lbl_message, 0, LV_STATE_DEFAULT, LV_COLOR_RED);

	lv_obj_align(lbl_message, odo_container, LV_ALIGN_OUT_TOP_LEFT, 4, 0);

	power_label = lv_label_create(screen, 0);
	lv_label_set_recolor(power_label, true);
#ifdef CONFIG_ESP_TFT_ILI9341
	lv_obj_set_style_local_text_font(power_label, 0, LV_STATE_DEFAULT, &lv_font_montserrat_14);

#else
	lv_obj_set_style_local_text_font(power_label, 0, LV_STATE_DEFAULT, &lv_font_montserrat_12);

#endif
	display_set_power_level(g_set_power_level);
	lv_obj_set_auto_realign(power_label, 1);
	// if(lv_disp_get_ver_res(disp) > 200) {
	// 	lv_obj_align(power_label, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -10);
	// 	//lv_obj_set_style(power_label, &font_28_style);

	// } else {
		lv_obj_align(power_label, linemeter, LV_ALIGN_OUT_TOP_MID, 0, 0);
	// }

	lv_obj_set_event_cb(power_label, [](struct _lv_obj_t * obj, lv_event_t event) {
		ESP_LOGI(__func__, "%d", event);
	});

	// lv_group_t* grp = lv_group_create();
	// lv_group_add_obj(grp, power_label);
	// //lv_group_add_obj(grp, linemeter);

	// lv_indev_set_group(btn_indev, grp);



	brake_icon = lv_img_create(screen, 0);
	lv_img_set_src(brake_icon, &brake);
	lv_obj_align(brake_icon, odo_container, LV_ALIGN_OUT_TOP_LEFT, 0, 0);
	lv_obj_set_hidden(brake_icon, true);

	lv_obj_set_style_local_image_recolor_opa(brake_icon, 0, LV_STATE_DEFAULT, 255);
	lv_obj_set_style_local_image_recolor(brake_icon, 0, LV_STATE_DEFAULT, LV_COLOR_RED);

	cruise_control_icon = lv_img_create(screen, 0);
	lv_img_set_src(cruise_control_icon, &cruise_control_img);
	lv_obj_align(cruise_control_icon, trp_info_container, LV_ALIGN_OUT_TOP_RIGHT, 0, 0);
	lv_obj_set_hidden(cruise_control_icon, true);
	lv_obj_set_style_local_image_recolor_opa(cruise_control_icon, 0, LV_STATE_DEFAULT, 255);
	lv_obj_set_style_local_image_recolor(cruise_control_icon, 0, LV_STATE_DEFAULT, LV_COLOR_GREEN);

	lv_obj_move_background(linemeter);
	lv_obj_move_foreground(lbl_mah);
	lv_obj_move_background(duty_gauge);

#ifdef CONFIG_LCD_TOUCH
	lv_obj_t *b = lv_btn_create(lv_disp_get_layer_top(disp), 0);
	static lv_style_t btn_label_style;
	lv_style_copy(&btn_label_style, lv_obj_get_style(b));
	btn_label_style.text.font = &lv_font_roboto_28;
	lv_obj_align(b, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 2, -2);
	lv_obj_t* lbl = lv_label_create(b, NULL);
	lv_obj_set_style(lbl, &btn_label_style);
    lv_label_set_text(lbl, LV_SYMBOL_LEFT);
	
	lv_obj_set_event_cb(b, [](struct _lv_obj_t * obj, lv_event_t event) {
		if(event == LV_EVENT_CLICKED) {
			display_set_power_level(g_set_power_level-1);
		}
		if(event == LV_EVENT_PRESSING) {
			printf("pressing\n");
			set_current_brake_rel(0.5f);
		}
		if(event == LV_EVENT_RELEASED) {
			set_current_brake_rel(0);

		}
	});

	b = lv_btn_create(lv_disp_get_layer_top(disp), 0);

	lv_obj_align(b, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -2, -2);
	lbl = lv_label_create(b, NULL);
	lv_obj_set_style(lbl, &btn_label_style);
    lv_label_set_text(lbl, LV_SYMBOL_RIGHT);
	lv_obj_set_event_cb(b, [](struct _lv_obj_t * obj, lv_event_t event) {
		printf("event %d\n", event);
		if(event == LV_EVENT_CLICKED) {
			display_set_power_level(g_set_power_level+1);
		}
		if(event == LV_EVENT_PRESSING) {
			printf("pressing\n");
			set_current_rel(0.2f);
		}
		if(event == LV_EVENT_RELEASED) {
			set_current_rel(0);

		}
	});
#endif

///


#ifdef DOUBLE_BUFFER
	xTaskCreate(&lvgl_flush_task, "gfx_flush", 1024, NULL, 7, NULL);
#endif

	lv_task_handler();
	
}

extern "C" {

extern uint32_t platform_time_ms();



void update_data(lv_task_t* task) {
	g_cur_display++;
	if(g_cur_display == DISPL_LAST) g_cur_display = 0;
	char text[16] = {};

	switch(g_cur_display) {
	case DISPL_TRIP:
		lv_label_set_static_text(lbl_trip, "TRP");
		snprintf(text, sizeof(text), "%.2f", g_trip);
		break;
	case DISPL_TRIPTIME:
		lv_label_set_static_text(lbl_trip, "TIM");
		snprintf(text, sizeof(text), "%02d:%02d", (int)g_triptime/60, (int)g_triptime%60);
		break;
	case DISPL_AVS:
		lv_label_set_static_text(lbl_trip, "AVS");
		snprintf(text, sizeof(text), "%02.1f", g_avgspeed);
		break;
	case DISPL_FET:
		lv_label_set_static_text(lbl_trip, "FET");
		snprintf(text, sizeof(text), "%.1f", g_mos_temp);
        break;
	case DISPL_WHKM:
		lv_label_set_static_text(lbl_trip, "Wh/Km");
		snprintf(text, sizeof(text), "%.1f", g_wh_km);
		break;
	}
	
	lv_label_set_text(lbl_trip_val, text);
}

void update_label_task(lv_task_t* task) {
	if(displ_message_updated) {
	    if(!displ_message_curr.empty()) {
		    lv_label_set_text(lbl_message, displ_message_curr.c_str());
		}
		displ_message_updated = false;
	}
}

void display_run() {
	TickType_t prevTick = 0;
	
	lv_task_create(update_data, 2500, LV_TASK_PRIO_MID, 0);
	lv_task_create(update_label_task, 100, LV_TASK_PRIO_MID, 0);

    while(1) {
    	uint32_t time_till_next_ms = lv_task_handler();
		vTaskDelay(time_till_next_ms / portTICK_PERIOD_MS);
    }
}

void display_show_menu() {
	LVGL_LOCK();

	if(ctrl_group->frozen) {

		lv_obj_t* top = lv_disp_get_layer_top(disp);
		static lv_obj_t* list;
		list = lv_list_create(top, 0);
		lv_obj_t* obj;

		lv_obj_set_event_cb(list, [](lv_obj_t * obj, lv_event_t event) {
			ESP_LOGI("lv_obj_set_event_cb", "list event %d\n", event);
		});

		obj = lv_list_add_btn(list, 0, "Throttle cal");
		lv_obj_set_event_cb(obj, [](lv_obj_t* obj, lv_event_t event) {
			if(event == LV_EVENT_CLICKED) {
				struct State {
					lv_obj_t* msgbox;
					lv_task_t* task;
					uint16_t thr_min = 0xffff;
					uint16_t thr_max = 0;
					uint8_t motor_armed;
				};

				static const char * btns[] ={"Reset", "OK", "Cancel", ""};

				lv_obj_t * msgbox1 = lv_msgbox_create(lv_disp_get_layer_top(disp), NULL);
				lv_msgbox_set_text(msgbox1, "Curr: 200\nMin: 230 Max: 789");
				lv_msgbox_add_btns(msgbox1, btns);
				//lv_obj_set_width(msgbox1, 200);
				lv_obj_set_event_cb(msgbox1, [](lv_obj_t* obj, lv_event_t event) {
					ESP_LOGI("msgbox1 event", "%d %d", event, lv_msgbox_get_active_btn(obj));
					assert(obj->user_data);

					State* state = (State*)obj->user_data;

					if(event == LV_EVENT_CLICKED) {
						uint8_t btn = lv_msgbox_get_active_btn(obj);
						if(btn == 0) {

							state->thr_max = 0;
							state->thr_min = 0xFFFF;							
						}
						if(btn == 1) {
							set_throttle_calibration(state->thr_min, state->thr_max);
							lv_msgbox_start_auto_close(obj, 0);
						}
						if(btn == 2 /* cancel */) {
							lv_msgbox_start_auto_close(obj, 0);
						}
					}
					if(event == LV_EVENT_DELETE) {
						lv_group_remove_all_objs(ctrl_group);
						lv_group_add_obj(ctrl_group, list);
						if(state->motor_armed)
							motor_arm();
						lv_task_del(state->task);

						delete state;
					}
				});
				lv_obj_align(msgbox1, NULL, LV_ALIGN_CENTER, 0, 0); /*Align to the corner*/

				lv_task_t* task = lv_task_create([](lv_task_t* task) {
					State* state = (State*)task->user_data;

					uint16_t adc = get_throttle_adc();
					if(adc>state->thr_max) state->thr_max = adc;
					if(adc<state->thr_min) state->thr_min = adc;

					char msg[128];
					sprintf(msg, "Curr: %d\nMin: %d Max: %d", adc, state->thr_min, state->thr_max);
					lv_msgbox_set_text(state->msgbox, msg);

				}, 100, LV_TASK_PRIO_HIGH, msgbox1);

				State* state = new State;
				state->msgbox = msgbox1;
				state->motor_armed = motor_disarm();
				state->task = task;
				task->user_data = state;
				msgbox1->user_data = state;
			
				lv_group_remove_all_objs(ctrl_group);
				lv_group_add_obj(ctrl_group, msgbox1);
				lv_group_focus_next(ctrl_group);

				//lv_group_focus_obj(lv_msgbox_get_btnm(msgbox1));
				//lv_group_set_editing(ctrl_group, 0);
				
			}
		});
		lv_list_add_btn(list, 0, "Motor Current");

		obj = lv_list_add_btn(list, 0, "Night mode");
		lv_btn_set_checkable(obj, true);
		if(night_mode) {
			lv_btn_set_state(obj, LV_BTN_STATE_CHECKED_RELEASED);
		}
		lv_obj_set_event_cb(obj, [](lv_obj_t* obj, lv_event_t event) {
			ESP_LOGI("night", "ev %d state %d", event, lv_btn_get_state(obj));
			if(event == LV_EVENT_VALUE_CHANGED) {
				if(lv_btn_get_state(obj)) {
					night_mode = true;
					LV_THEME_DEFAULT_INIT(lv_theme_get_color_primary(), lv_theme_get_color_secondary(),
						LV_THEME_MATERIAL_FLAG_DARK,
						lv_theme_get_font_small(), lv_theme_get_font_normal(), lv_theme_get_font_subtitle(), lv_theme_get_font_title());
				} else {
					night_mode = false;
					LV_THEME_DEFAULT_INIT(lv_theme_get_color_primary(), lv_theme_get_color_secondary(),
						LV_THEME_MATERIAL_FLAG_LIGHT,
						lv_theme_get_font_small(), lv_theme_get_font_normal(), lv_theme_get_font_subtitle(), lv_theme_get_font_title());	
				}
			}
		});

		obj = lv_list_add_btn(list, 0, "Close");
		lv_obj_set_event_cb(obj, [](lv_obj_t* obj, lv_event_t event) {
			if(event == LV_EVENT_CLICKED) {
				lv_group_remove_all_objs(ctrl_group);
				lv_obj_del_async(list);
				lv_group_focus_freeze(ctrl_group, true);
				list = NULL;
			}
		});



		lv_group_focus_freeze(ctrl_group, false);

		lv_group_remove_all_objs(ctrl_group);
		lv_group_add_obj(ctrl_group, list);	
		lv_group_set_editing(ctrl_group, false);
	}

	LVGL_UNLOCK();
}

void display_set_cruise_control_cb(void(*func)(uint8_t)) {
	cruise_control_func = func;
}

void display_set_power_level_cb(void(*func)(int8_t)) {
	power_lvl_changed_fn = func;
}

void display_set_power_level(int8_t power_level) {
	if(power_level > 4) power_level = 4; else
	if(power_level < -4) power_level = -4;
	g_set_power_level = power_level;
	ESP_LOGI(__func__, "set %d", power_level);

	char buf[128];
	make_power_label(buf, g_set_power_level);
	LVGL_LOCK();
	lv_label_set_text(power_label, buf);
	LVGL_UNLOCK();

	if(power_lvl_changed_fn)
		power_lvl_changed_fn(g_set_power_level);
}

void display_set_wh_km(float wh_km) {
	g_wh_km = wh_km;
}

void display_set_mot_current_minmax(float min, float max) {
	LVGL_LOCK();
	if(min != max) {
		lv_bar_set_range(bar_power, min, max);
	}
	LVGL_UNLOCK();
}

void display_set_mot_current(float current) {
	ON_CHANGED(lroundf(current), {
		LVGL_LOCK();
		lv_bar_set_value(bar_power, current, 0);
		lv_label_set_text_fmt(lbl_mot_curr, "%dA", value);
		LVGL_UNLOCK();
	})

}

void display_set_curr_power(float power) {
	ON_CHANGED(power, {
		LVGL_LOCK();
		lv_label_set_text_fmt(lbl_pow, "%.0fW", power);
		LVGL_UNLOCK();
	});
}

void display_set_trip(float dist) {
	g_trip = dist;
	if(g_cur_display == DISPL_TRIP) {
		ON_CHANGED(dist, {
			LVGL_LOCK();
			lv_label_set_text_fmt(lbl_trip_val, "%.2f", dist);
			lv_obj_realign(lbl_trip);
			LVGL_UNLOCK();
		});
	}
}


void display_set_odo(float dist) {
	ON_CHANGED((int)floor(dist),
	{
		LVGL_LOCK();
		lv_label_set_text_fmt(lbl_odo, "%05d", value);
		LVGL_UNLOCK();
	});

}

void display_show_message_timed(const char* message, int timeout) {
	if(strcmp(displ_message_curr.c_str(), message) != 0) {
		displ_message_curr = message;
		displ_message_updated = true;

		lv_task_t* task = lv_task_create([](lv_task_t* task) {
			displ_message_curr = "";
			displ_message_updated = true;
			lv_task_del(task);
		}, timeout, LV_TASK_PRIO_MID, 0);
		
	}
	// if(!message) {
	// 	lv_label_set_text(lbl_message, "");
	// } else {
	// 	lv_label_set_text_fmt(lbl_message, "#ff0000 %s#", message);
	// }
}


void display_show_message(const char* message) {
	if(strcmp(displ_message_curr.c_str(), message) != 0) {
		displ_message_curr = message;
		displ_message_updated = true;
	}
	// if(!message) {
	// 	lv_label_set_text(lbl_message, "");
	// } else {
	// 	lv_label_set_text_fmt(lbl_message, "#ff0000 %s#", message);
	// }
}

void display_set_duty(float duty) {
	ON_CHANGED(lroundf(fabsf(duty*100)), {
			LVGL_LOCK();
			lv_gauge_set_value(duty_gauge, 0, value);
			LVGL_UNLOCK();
	});
}

void display_set_mos_temp(float temp) {
	g_mos_temp = temp;
	if(g_cur_display == DISPL_FET) {
		ON_CHANGED(temp, {
			LVGL_LOCK();
			lv_label_set_text_fmt(lbl_trip_val, "%.1f", temp);
			LVGL_UNLOCK();
		});
	}
}

void display_set_bat_limits(float cut_start, float cut_end) {
	ESP_LOGI(__func__, "start:%f end:%f", cut_start, cut_end);
	bat_cut_start = cut_start;
	bat_cut_end = cut_end;
}

void display_set_bat_info(float volts, float amps) {
	ON_CHANGED(volts, {
		LVGL_LOCK();
		if(volts < bat_cut_start) {
			lv_label_set_text_fmt(lbl_volts, "#ff0000 %02.1fV#", volts);
		} else {
			lv_label_set_text_fmt(lbl_volts, "%02.1fV", volts);
		}
		LVGL_UNLOCK();
	});

	ON_CHANGED(amps, {
		LVGL_LOCK();
		lv_label_set_text_fmt(lbl_amps, "%02.1fA", amps);
		LVGL_UNLOCK();
	});
}

void display_set_energy(float ah_in, float ah_out) {
	ON_CHANGED_2(ah_in, ah_out, {
		LVGL_LOCK();
		lv_label_set_text_fmt(lbl_mah, "mAh\nDSG%05d\nCHG%05d", lroundf(ah_out*1000.0f), lroundf(ah_in*1000.0f));
		LVGL_UNLOCK();
	})

}

void display_set_speed(float speed) {
	ON_CHANGED(lroundf(fabsf(speed)), {
		LVGL_LOCK();
		lv_linemeter_set_value(linemeter, value);
		lv_label_set_text_fmt(lbl_speed, "%02d", value);
		LVGL_UNLOCK();
	});
}

void display_set_triptime(int triptime) {
	if(g_cur_display == DISPL_TRIPTIME) {
		ON_CHANGED(triptime/1000.0f, {
			g_triptime = value;
			LVGL_LOCK();
			lv_label_set_text_fmt(lbl_trip_val, "%02d:%02d", (int)value/60, (int)value%60);
			LVGL_UNLOCK();
		})
	}
}

void display_set_avgspeed(float avgspeed) {
	if(g_cur_display == DISPL_AVS) {
		ON_CHANGED(avgspeed, {
			g_avgspeed = value;
			LVGL_LOCK();
			lv_label_set_text_fmt(lbl_trip_val, "%02.1f", avgspeed);
			LVGL_UNLOCK();
		})
	}
}

void display_set_connected(bool connected) {
	ON_CHANGED(connected, {
		LVGL_LOCK();
		if(connected) {
			lv_obj_set_style_local_opa_scale(lbl_wifisymbol, 0, LV_STATE_DEFAULT, 255);
		} else {
			lv_obj_set_style_local_opa_scale(lbl_wifisymbol, 0, LV_STATE_DEFAULT, 64);

		}
		LVGL_UNLOCK();
	});
}

void display_set_brake_icon(uint8_t show) {
	ON_CHANGED(show, {
		LVGL_LOCK();
		lv_obj_set_hidden(brake_icon, !show);
		LVGL_UNLOCK();
	});
}

void display_set_cruise_icon(uint8_t show) {
	ON_CHANGED(show, {
		LVGL_LOCK();
		lv_obj_set_hidden(cruise_control_icon, !show);
		LVGL_UNLOCK();
	});
}

}

