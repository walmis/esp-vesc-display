#pragma GCC optimize("O3")


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
#include "spi.h"

//#define GPIO_TFT_LED (gpio_num_t)4
//#define GPIO_TFT_RST (gpio_num_t)0
//#define GPIO_TFT_DC  (gpio_num_t)15
//#define GPIO_TFT_CS  (gpio_num_t)2

#define DOUBLE_BUFFER

#include "TFT_22_ILI9225.h"

#ifdef DOUBLE_BUFFER
struct flush_data {
	lv_disp_drv_t * disp;
	lv_area_t area;
	lv_color_t * color_p;
};

static xQueueHandle flush_q;
#endif

static SemaphoreHandle_t lvgl_mutex;
static lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];                     /*Declare a buffer for 10 lines*/
#ifdef DOUBLE_BUFFER
static lv_color_t buf2[LV_HOR_RES_MAX * 10];                     /*Declare a buffer for 10 lines*/
#endif




#define LVGL_LOCK()   xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY );
#define LVGL_UNLOCK() xSemaphoreGiveRecursive(lvgl_mutex );

#define ON_CHANGED(x, y) { static typeof(x) _value; typeof(x) value = x; if(_value != value) { _value = value; y }}

#define ON_CHANGED_2(x1, x2, y) static typeof(x1) _value_1; \
								static typeof(x2) _value_2;	\
								typeof(x1) value_1 = x1;\
								typeof(x2) value_2 = x2;\
								if(value_1 != _value_1 || value_2 != _value_2) { _value_1 = value_1; _value_2 = value_2; y }



#ifdef ESP_TFT_ILI9225
IRAM_ATTR
class TFT : public TFT_22_ILI9225 {
public:

	void begin() {
		gpio_config_t cfg = {};
		cfg.intr_type = GPIO_INTR_DISABLE;
		cfg.mode = GPIO_MODE_OUTPUT;
		cfg.pin_bit_mask = BIT(CONFIG_GPIO_TFT_CS) | BIT(CONFIG_GPIO_TFT_DC) | BIT(CONFIG_GPIO_TFT_RST) | BIT(CONFIG_GPIO_TFT_LED);
		gpio_config(&cfg);

		gpio_set_level((gpio_num_t) CONFIG_GPIO_TFT_LED, 1);

		spi_init();

		TFT_22_ILI9225::begin();
	}

	void _writeCommand16(uint16_t command) {
		spi_writeCommand16(command);
	}

	void _writeData16(uint16_t data) {
		spi_writeData16(data);
	}

private:
	void assertCS(bool state)  { gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_CS, state); }
	void assertDC(bool state)  { gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_DC, state); }
	void assertRST(bool state) { gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_RST, state); }



	void startWrite() override {}

	void endWrite() override {
		spi_flushdata();
	}
};
static TFT tft;

#else 



#endif


#ifdef DOUBLE_BUFFER
void IRAM_ATTR lvgl_flush_task(void* parm) {
	struct flush_data args;
	ESP_LOGI(__func__, "start");

	while(1) {
		xQueueReceive(flush_q, &args, portMAX_DELAY);
		//ESP_LOGI(__func__, "flush");
		//printf("win2 x1:%d y1:%d x2:%d y2:%d %p\n", args.area.x1, args.area->y1, args.area->x2, args.area->y2, args.color_p);

#ifdef CONFIG_ESP_TFT_ILI9341
		ILI9341_setWindow(args.area.x1, args.area.y1, args.area.x2, args.area.y2);
#elif CONFIG_ESP_TFT_ILI9225
		tft._setWindow(args.area.x1, args.area.y1, args.area.x2, args.area.y2, L2R_TopDown);
#endif
		spi_beginTransaction(CONFIG_GPIO_TFT_CS);
		int32_t x, y;
		for(y = args.area.y1; y <= args.area.y2; y++) {
			for(x = args.area.x1; x <= args.area.x2; x++) {
				spi_writeData16(args.color_p->full);
				args.color_p++;
			}
		}
		spi_flushdata();
		spi_endTransaction();

		lv_disp_flush_ready(args.disp);         /* Indicate you are ready with the flushing*/
	}
}
#endif

static IRAM_ATTR void lvgl_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p) {
#ifdef DOUBLE_BUFFER
	struct flush_data args;
	args.disp = disp;
	args.area = *area;
	args.color_p = color_p;

	//ESP_LOGI(__func__, "flush");
	xQueueSend(flush_q, &args, portMAX_DELAY);
#else
	tft._setWindow(area->x1, area->y1, area->x2, area->y2, L2R_TopDown);

    int32_t x, y;
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
        	spi_writeData16(color_p->full);
            color_p++;
        }
    }

    lv_disp_flush_ready(disp);         /* Indicate you are ready with the flushing*/
#endif
}




void monitor_cb(struct _disp_drv_t * disp_drv, uint32_t time, uint32_t px) {
	ESP_LOGI(__func__, "refreshed %dpx in %dms", px, time);
}

extern "C"
void vApplicationTickHook() {
	lv_tick_inc(portTICK_PERIOD_MS);
}

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
static lv_obj_t *lmeter;
static lv_obj_t* duty_gauge;
static lv_theme_t * theme;
static lv_obj_t* lbl_volts;
static lv_obj_t* lbl_amps;
static lv_obj_t* lbl_wifisymbol;
static lv_obj_t* lbl_message;

extern "C"
void display_setup() {
	lvgl_mutex = xSemaphoreCreateRecursiveMutex();
#ifdef DOUBLE_BUFFER
	flush_q = xQueueCreate(1, sizeof(struct flush_data));
#endif
	spi_init();

#ifdef CONFIG_ESP_TFT_ILI9341
	ILI9341_init();
#elif CONFIG_ESP_TFT_ILI9225
    tft.begin();
    tft.setOrientation(3);
#endif

    lv_init();

#ifdef DOUBLE_BUFFER
    lv_disp_buf_init(&disp_buf, buf, buf2, LV_HOR_RES_MAX * 10);    /*Initialize the display buffer*/
#else
    lv_disp_buf_init(&disp_buf, buf, 0, LV_HOR_RES_MAX * 10);    /*Initialize the display buffer*/
#endif
    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.flush_cb = lvgl_flush;    /*Set your driver function*/
    //disp_drv.monitor_cb = monitor_cb;
    disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
    lv_disp_t* disp = lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

    //theme = lv_theme_material_init(0, NULL);
    theme = lv_theme_material_init(20, 0);
    lv_theme_set_current(theme);

	lv_obj_t* top_layer = lv_disp_get_layer_top(disp);

    lv_obj_set_style(top_layer, theme->style.scr);

    /**
     * Get a pointer to the theme
     * @return pointer to the theme
     */
    bar_power = lv_bar_create(top_layer, NULL);
    lv_obj_set_height(bar_power, 10);
    lv_bar_set_range(bar_power, -100, 100);
    lv_bar_set_value(bar_power, 0, 0);
    lv_bar_set_sym(bar_power, true);
    lv_obj_align(bar_power, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -40);


    lbl_speed = lv_label_create(top_layer, NULL);

    static lv_style_t font_large_num;
    lv_style_copy(&font_large_num, lv_obj_get_style(lbl_speed));
    font_large_num.text.font = &iosevka_num;

    static lv_style_t font_22_style;
    lv_style_copy(&font_22_style, lv_obj_get_style(lbl_speed));
    font_22_style.text.font = &lv_font_roboto_22;

    static lv_style_t font_20_style;
    lv_style_copy(&font_20_style, lv_obj_get_style(lbl_speed));
    font_20_style.text.font = &iosevka_20;

    static lv_style_t font_iosevka_14_style;
    lv_style_copy(&font_iosevka_14_style, lv_obj_get_style(lbl_speed));
    font_iosevka_14_style.text.font = &iosevka_14;

    static lv_style_t font_10_style;
    lv_style_copy(&font_10_style, lv_obj_get_style(lbl_speed));
    font_10_style.text.font = &iosevka_10;


    lv_label_set_style(lbl_speed, LV_LABEL_STYLE_MAIN, &font_large_num);
    lv_label_set_text(lbl_speed, "00");
    lv_obj_align(lbl_speed, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t* lbl_kmh = lv_label_create(top_layer, NULL);
    //lv_label_set_style(lbl, LV_LABEL_STYLE_MAIN, &font_test_style);
    lv_label_set_text(lbl_kmh, "km/h");
    lv_obj_align(lbl_kmh, NULL, LV_ALIGN_CENTER, 0, 28);

    lmeter = lv_lmeter_create(top_layer, NULL);
    lv_lmeter_set_range(lmeter, 0, 50);
    lv_obj_set_height(lmeter, 120);
    lv_obj_set_width(lmeter, 100);
    lv_obj_align(lmeter, NULL, LV_ALIGN_CENTER, 0, 0);

    {
    	lv_obj_t* cont = lv_cont_create(top_layer, 0);
		lbl_func = lv_label_create(cont, NULL);
		//lv_label_set_style(lbl_func, LV_LABEL_STYLE_MAIN, &font_22_style);
		lv_label_set_text(lbl_func, "ODO");
		lv_obj_align(lbl_func, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 2, -2);


		lbl_odo = lv_label_create(cont, NULL);
		lv_label_set_style(lbl_odo, LV_LABEL_STYLE_MAIN, &font_20_style);
		lv_label_set_text(lbl_odo, "00000");
		lv_obj_align(lbl_odo, lbl_func, LV_ALIGN_OUT_RIGHT_MID, 2, 0);

		lv_cont_set_style(cont, LV_CONT_STYLE_MAIN, theme->style.bg);
		lv_cont_set_layout(cont, LV_LAYOUT_ROW_M);
		lv_cont_set_fit2(cont, LV_FIT_TIGHT, LV_FIT_TIGHT);
		lv_obj_align(cont, 0, LV_ALIGN_IN_BOTTOM_LEFT, 0, 4);
		lv_obj_set_auto_realign(cont, 1);
    }

    {
    	lv_obj_t* cont = lv_cont_create(top_layer, 0);

		lbl_trip = lv_label_create(cont, NULL);
		//lv_label_set_style(lbl_func, LV_LABEL_STYLE_MAIN, &font_22_style);
		lv_label_set_text(lbl_trip, "TRP");

		lbl_trip_val = lv_label_create(cont, NULL);
		lv_label_set_style(lbl_trip_val, LV_LABEL_STYLE_MAIN, &font_20_style);
		lv_label_set_text(lbl_trip_val, "0.00");
		//lv_obj_align(lbl_trip_val, 0, LV_ALIGN_IN_BOTTOM_RIGHT, -2, 0);

		lv_cont_set_style(cont, LV_CONT_STYLE_MAIN, theme->style.bg);
		lv_cont_set_layout(cont, LV_LAYOUT_ROW_M);
		lv_cont_set_fit2(cont, LV_FIT_TIGHT, LV_FIT_TIGHT);
		lv_obj_align(cont, 0, LV_ALIGN_IN_BOTTOM_RIGHT, 0, 4);
		lv_obj_set_auto_realign(cont, 1);

    }


//    lbl_bat = lv_label_create(lv_disp_get_layer_top(disp), NULL);
//    //lv_label_set_style(lbl_trip_val, LV_LABEL_STYLE_MAIN, &font_22_style);
//    lv_label_set_text(lbl_bat, LV_SYMBOL_BATTERY_3 " 44.1V 2.3A");
//    lv_obj_align(lbl_bat, 0, LV_ALIGN_IN_TOP_LEFT, 2, -2);
//    lv_label_set_recolor(lbl_bat, 1);

    lbl_mah = lv_label_create(top_layer, NULL);
    lv_label_set_style(lbl_mah, LV_LABEL_STYLE_MAIN, &font_iosevka_14_style);
    lv_label_set_text_fmt(lbl_mah, "mAh\nDSG%05d\nCHG%05d", 0, 0);
    lv_label_set_align(lbl_mah, LV_LABEL_ALIGN_RIGHT);

    lv_obj_align(lbl_mah, 0, LV_ALIGN_IN_TOP_RIGHT, -2, 0);

    lv_obj_t* cont = lv_cont_create(top_layer, NULL);
    lv_cont_set_fit2(cont, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_set_width(cont, 60);
    static lv_style_t cont_style;
    lv_style_copy(&cont_style, lv_obj_get_style(cont));
    cont_style.body.padding.bottom = 4;
    cont_style.body.padding.top = 4;
    cont_style.body.padding.left = 2;
    cont_style.body.padding.right = 2;
    lv_obj_set_style(cont, &cont_style);

    lbl_pow = lv_label_create(cont, NULL);
    lv_label_set_style(lbl_pow, LV_LABEL_STYLE_MAIN, &font_20_style);
    lv_label_set_text(lbl_pow, "0W");
    lv_obj_align(lbl_pow, 0, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_auto_realign(lbl_pow, 1);

    lv_obj_align(cont, 0, LV_ALIGN_IN_LEFT_MID, 2, 0);

    cont = lv_cont_create(top_layer, NULL);
    lv_obj_set_style(cont, &cont_style);


    lbl_mot_curr = lv_label_create(cont, NULL);
    lv_label_set_style(lbl_mot_curr, LV_LABEL_STYLE_MAIN, &font_20_style);
    lv_label_set_text(lbl_mot_curr, "0A");
    lv_obj_set_auto_realign(lbl_mot_curr, 1);
    //lv_cont_set_fit(cont, LV_FIT_TIGHT);
    lv_cont_set_fit2(cont, LV_FIT_NONE, LV_FIT_TIGHT);
    lv_obj_set_width(cont, 60);
    lv_obj_align(lbl_mot_curr, 0, LV_ALIGN_CENTER, 0, 0);

    lv_obj_align(cont, 0, LV_ALIGN_IN_RIGHT_MID, -2, 0);

    /* status bar */
    lv_obj_t* statusbar = lv_cont_create(top_layer, NULL);
    lv_cont_set_style(statusbar, LV_CONT_STYLE_MAIN, theme->style.bg);

    lbl_wifisymbol = lv_label_create(statusbar, NULL);
    //lv_label_set_style(lbl, LV_LABEL_STYLE_MAIN, &font_test_style);
    lv_label_set_text(lbl_wifisymbol, LV_SYMBOL_WIFI);
    lv_obj_set_opa_scale(lbl_wifisymbol, 64);
    lv_obj_set_opa_scale_enable(lbl_wifisymbol, 1);

    lv_obj_t* lbl_battsymbol = lv_label_create(statusbar, NULL);
    //lv_label_set_style(lbl, LV_LABEL_STYLE_MAIN, &font_test_style);
    lv_label_set_text(lbl_battsymbol, LV_SYMBOL_BATTERY_3);

    lbl_volts = lv_label_create(statusbar, NULL);
    //lv_label_set_style(lbl_volts, LV_LABEL_STYLE_MAIN, &font_20_style);
    lv_label_set_text(lbl_volts, "0.0V");
    lv_label_set_recolor(lbl_volts, 1);


    lbl_amps = lv_label_create(statusbar, NULL);
    //lv_label_set_style(lbl_amps, LV_LABEL_STYLE_MAIN, &font_20_style);
    lv_label_set_text(lbl_amps, "0.0A");

    lv_cont_set_layout(statusbar, LV_LAYOUT_ROW_M);
    lv_cont_set_fit(statusbar, LV_FIT_TIGHT);

    lv_obj_align(statusbar, 0, LV_ALIGN_IN_TOP_LEFT, 2, -2);

    /* duty gauge */
    duty_gauge = lv_gauge_create(top_layer, NULL);
    lv_gauge_set_range(duty_gauge, 0, 100);
    lv_obj_set_width(duty_gauge, 60);
    lv_obj_set_height(duty_gauge, 60);
    lv_gauge_set_scale(duty_gauge, 180, 10, 0);
    lv_gauge_set_value(duty_gauge, 0, 0);
    lv_obj_align(duty_gauge, 0, LV_ALIGN_IN_LEFT_MID, 2, -34);
    /* ********** */

	lbl_message = lv_label_create(top_layer, NULL);
	//lv_label_set_style(lbl_message, LV_LABEL_STYLE_MAIN, &font_10_style);
	lv_label_set_recolor(lbl_message, true);
	lv_label_set_static_text(lbl_message, "");
	lv_obj_align(lbl_message, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 4, -21);

#ifdef DOUBLE_BUFFER
	xTaskCreate(&lvgl_flush_task, "gfx_flush", 1024, NULL, 7, NULL);
#endif

	lv_task_handler();
}

extern "C" {

enum {
	DISPL_TRIP,
	DISPL_TRIPTIME,
	DISPL_AVS,
	DISPL_FET,
	DISPL_LAST
};

extern uint32_t platform_time_ms();

static uint8_t g_cur_display = DISPL_TRIP;

static float g_triptime;
static float g_avgspeed;
static float g_trip;
static float g_mos_temp;

static float bat_cut_start;
static float bat_cut_end;
int ctr;

static uint32_t g_tm_prev_change;

void display_run() {
	TickType_t prevTick = 0;
    while(1) {
    	LVGL_LOCK();
    	lv_task_handler();
    	ctr++;

        if(platform_time_ms() - g_tm_prev_change > 2500) {
        	g_cur_display++;
        	if(g_cur_display == DISPL_LAST) g_cur_display = 0;

        	switch(g_cur_display) {
        	case DISPL_TRIP:
        		lv_label_set_static_text(lbl_trip, "TRP");
        		lv_label_set_text_fmt(lbl_trip_val, "%.2f", g_trip);
        		break;
        	case DISPL_TRIPTIME:
        		lv_label_set_static_text(lbl_trip, "TIM");
        		lv_label_set_text_fmt(lbl_trip_val, "%02d:%02d", (int)g_triptime/60, (int)g_triptime%60);
        		break;
        	case DISPL_AVS:
        		lv_label_set_static_text(lbl_trip, "AVS");
        		lv_label_set_text_fmt(lbl_trip_val, "%02.1f", g_avgspeed);
        		break;
        	case DISPL_FET:
        		lv_label_set_static_text(lbl_trip, "FET");
        		lv_label_set_text_fmt(lbl_trip_val, "%.1f", g_mos_temp);
        	}

        	g_tm_prev_change = platform_time_ms();
        }

    	LVGL_UNLOCK();

    	vTaskDelayUntil(&prevTick, 10 / portTICK_PERIOD_MS);
    }
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
	ON_CHANGED(lroundf(dist),
	{
		LVGL_LOCK();
		lv_label_set_text_fmt(lbl_odo, "%05d", value);
		LVGL_UNLOCK();
	});

}

void display_show_message(const char* message) {
	ON_CHANGED(message, {
		if(!message) {
			lv_label_set_text(lbl_message, "");
		} else {
			lv_label_set_text_fmt(lbl_message, "#ff0000 %s#", message);
		}
	});
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
		char s_volts[16];

		if(volts < bat_cut_start) {
			snprintf(s_volts, sizeof(s_volts), "#ff0000 %02.1fV#", volts);
		} else {
			snprintf(s_volts, sizeof(s_volts), "%02.1fV", volts);
		}
		LVGL_LOCK();
		lv_label_set_text(lbl_volts, s_volts);
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
		lv_lmeter_set_value(lmeter, value);
		lv_label_set_text_fmt(lbl_speed, "%02d", value);
		LVGL_UNLOCK();
	});
}

void display_set_triptime(uint32_t triptime) {
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
		    lv_obj_set_opa_scale(lbl_wifisymbol, 255);
		} else {
		    lv_obj_set_opa_scale(lbl_wifisymbol, 64);
		}
		LVGL_UNLOCK();
	})
}

}

