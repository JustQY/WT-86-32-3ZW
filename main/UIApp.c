/* Mesh Internal Communication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "esp_log.h"
#include "screen_driver.h"
#include "lvgl.h"
#include "UIApp.h"
#include "ili9488.h"
#include "time.h"
#include "GSL2038.h"
#include "lvgl/demos/lv_demos.h"

static const char *TAG = "UI";

#define LCD_WR_SCK 39
#define LCD_RS_DC 38
#define LCD_CS -1
#define LCD_RST 37
#define LCD_BL 0
#define LCD_D0 36
#define LCD_D1 35
#define LCD_D2 34
#define LCD_D3 33
#define LCD_D4 21
#define LCD_D5 18
#define LCD_D6 17
#define LCD_D7 16
#define SWAP_DATA 1
#define LCD_DRIVER SCREEN_CONTROLLER_ILI9488
#define SC_DIR SCR_DIR_RLTB

xQueueHandle QueueUI;

static scr_driver_t g_lcd;
static int BackLightTimeout = 0;

void disp_driver_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
   if (!BackLightTimeout)
   {
      memset(color_map, 0, ((area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) * sizeof(lv_color_t)));
   }
   //ESP_LOGE(TAG,"x:%d,y:%d",area->x1+area->x2,area->y1+area->y2);
   lcd_ili9488_draw_bitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, (uint16_t *) color_map);
   // lcd_st7796_draw_bitmap(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, color_map);
   lv_disp_flush_ready(drv);
}

void my_read_cb(struct _lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
   //bool res = false
   static bool isPress = 0;
   uint16_t x = 0, y = 0;
   if (GSL2308GetPoint(&x, &y))
   {
      data->point.x = x / 3;
      data->point.y = (uint16_t)((y-24) / 2);
      if (data->point.x > 320)
         data->point.x = 320;
      if(data->point.y > 320)
         data->point.y = 320;
      //ESP_LOGI(TAG, "Touch x:%d y:%d", data->point.x, data->point.y);
      data->state = LV_INDEV_STATE_PR;
   }
   else
   {
      data->point.x = 0;
      data->point.y = 0;
      data->state = LV_INDEV_STATE_REL;
   }
   if (BackLightTimeout == 0)
   {
      lv_scr_load(lv_scr_act());
      isPress = 1;
   }
   BackLightTimeout = 60000;//UIParameter.BackLightOffTime;
   gpio_set_level(LCD_BL, 1);
   if (isPress == 1)
   {
      data->state = LV_INDEV_STATE_REL;
   }
#if 0
   bool res = that_read_cb(indev_drv, data);
   static lv_coord_t last_x = 0;
   static lv_coord_t last_y = 0;
   static bool isPress = 0;
   /*Set the last pressed coordinates*/
   if (last_x != data->point.x || last_y != data->point.y)
   {
       if (isPress == 1)
       {
           isPress = 0;
       }
   }
   last_x = data->point.x;
   last_y = data->point.y;
   if (data->state == LV_INDEV_STATE_PR)
   {
       if (BackLightTimeout == 0)
       {
           lv_scr_load(lv_scr_act());
           isPress = 1;
       }
       BackLightTimeout = UIParameter.BackLightOffTime;
       gpio_set_level(LCD_BL, 1);
       if (isPress == 1)
       {
           data->state = LV_INDEV_STATE_REL;
       }
   }
#endif
}

void TaskGUI()
{
   while (1)
   {
      if (BackLightTimeout)
      {
         BackLightTimeout--;
         if (BackLightTimeout == 0)
         {
            lv_scr_load(lv_scr_act());
            gpio_set_level(LCD_BL, 0);
         }
      }
      lv_task_handler();
      lv_tick_inc(1);
      vTaskDelay(1);
   }
}

void UIInit()
{
   esp_err_t ret = ESP_OK;

   lv_init();
   
   lv_color_t *buf1 = heap_caps_malloc((320 * 320 * sizeof(lv_color_t)), MALLOC_CAP_SPIRAM);
   lv_color_t *buf2 = heap_caps_malloc((320 * 320 * sizeof(lv_color_t)), MALLOC_CAP_SPIRAM);
   static lv_disp_draw_buf_t  disp_buf;
   lv_disp_draw_buf_init(&disp_buf, buf1, buf2, 320 * 320);

#if (defined CONFIG_WT154_S2MI1) || defined(CONFIG_LCD01TG_A_SP)
   gpio_reset_pin(LCD_RD);
   gpio_set_direction(LCD_RD, GPIO_MODE_OUTPUT);
   gpio_set_level(LCD_RD, 1);
#endif
   i2s_lcd_config_t i2s_lcd_cfg =
         {
               .data_width = 8,
               .pin_data_num = {
                     LCD_D0,
                     LCD_D1,
                     LCD_D2,
                     LCD_D3,
                     LCD_D4,
                     LCD_D5,
                     LCD_D6,
                     LCD_D7,
                     // 1, 2, 3, 4, 5, 6, 7, 8,
               },
               .pin_num_cs = LCD_CS,
               .pin_num_wr = LCD_WR_SCK,
               .pin_num_rs = LCD_RS_DC,
               .clk_freq = 20000000,
               .i2s_port = I2S_NUM_0,
               .buffer_size = 32000,
               .swap_data = SWAP_DATA,
         };

   scr_interface_driver_t *iface_drv;
   scr_interface_create(SCREEN_IFACE_8080, &i2s_lcd_cfg, &iface_drv);
   ret = scr_find_driver(LCD_DRIVER, &g_lcd);
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "screen find failed");
      return;
   }
   scr_controller_config_t lcd_cfg =
         {
               .interface_drv = iface_drv,
               .pin_num_rst = LCD_RST,
               .pin_num_bckl = LCD_BL,
               .rst_active_level = 0,
               .bckl_active_level = 1,
               .offset_hor = 0,
               .offset_ver = 0,
               .width = 320,
               .height = 320,
               .rotate = SC_DIR,
         };

   static lv_disp_drv_t disp_drv;
   lv_disp_drv_init(&disp_drv);
   disp_drv.flush_cb = disp_driver_flush;
   disp_drv.hor_res = 320;
   disp_drv.ver_res = 320;
   disp_drv.draw_buf  = &disp_buf;
   disp_drv.full_refresh = 1;

   lv_disp_drv_register(&disp_drv);
   ret = g_lcd.init(&lcd_cfg);

   GSL2038Init();

   static lv_indev_drv_t indev_drv;
   lv_indev_drv_init(&indev_drv);
   indev_drv.type = LV_INDEV_TYPE_POINTER;
   indev_drv.read_cb = my_read_cb;
   lv_indev_drv_register(&indev_drv);

   BackLightTimeout = 60000;//UIParameter.BackLightOffTime;
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "screen initialize failed");
      return;
   }

#if 0
   setup_ui(&uiMain);
   events_init(&uiMain);
   custom_init(&uiMain);
#else
   lv_demo_keypad_encoder();
#endif

   xTaskCreatePinnedToCore(TaskGUI, "gui", 4096, NULL, 15, NULL, 1);
   // xTaskCreate(TaskGUI, "tft", 4 * 1024,NULL, 15, NULL);
}
