/**************************************************************************/
/*!
    @file     GSL2038.h

    @thanks to Skallwar for the source code this lib is bassed on : https://github.com/Skallwar/GSL2038
*/

#include "stdint.h"
#include "string.h"
#include "esp_log.h"
#include "freertos/freertos.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "GSL2038.h"
#include "gsl2038firmware.h"

#define TAG             "gsl2038"

#define IICPortNumber            I2C_NUM_0

#define TOUCH_SCL 14
#define TOUCH_SDA 15
#define TOUCH_RST 12
#define TOUCH_IRQ 13

xQueueHandle QueueTouch = NULL;

void GSL2038Reset()
{
   uint8_t pack[] = {0xE4, 0x04, 0xbc, 0x00, 0xbd, 0x00, 0xbe, 0x00, 0xbf, 0x00};
   int ret = i2c_master_write_to_device(IICPortNumber, I2CADDR, pack, sizeof(pack),
                                        100);
   if (ret)
      ESP_LOGE(TAG, "error");
}

void GSL2038Start()
{
   uint8_t write_buf[2] = {0xe0, 0};

   int ret = i2c_master_write_to_device(IICPortNumber, I2CADDR, write_buf, sizeof(write_buf),
                                        0);
   if (ret)
      ESP_LOGE(TAG, "error");
}

void GSL2038TransmitData(uint8_t REG, uint8_t data[], uint16_t length)
{
   uint8_t buff[10];
   buff[0] = REG;
   memcpy(&buff[1], data, length);
   int ret = i2c_master_write_to_device(IICPortNumber, I2CADDR, buff, length + 1, 0);
   if (ret)
      ESP_LOGE(TAG, "error 001");
}

uint8_t GSL2038ReadTouchPoint(TouchPointType points[5])
{
   uint8_t buffer[24] = {0};
   uint8_t readAddr = DATA_REG;
   int ret = i2c_master_write_read_device(IICPortNumber, I2CADDR, &readAddr,
                                          1, buffer, sizeof(buffer), 0);
   if (ret)
      ESP_LOGE(TAG, "error");

   uint8_t count = buffer[0];
   if(count > 5)
      return 0;
   for (int i = 0; i < count; i++)
   {
      points[i].X = ((((uint32_t) buffer[(i * 4) + 5]) << 8) | (uint32_t) buffer[(i * 4) + 4]) &
                              0x00000FFF; // 12 bits of X coord
      points[i].Y =
            ((((uint32_t) buffer[(i * 4) + 7]) << 8) | (uint32_t) buffer[(i * 4) + 6]) & 0x00000FFF;
      points[i].FingerID = (uint32_t) buffer[(i * 4) + 7] >> 4; // finger that did the touch
   }

   return count;
}

void GSL2038LoadFirmware()
{
   uint8_t addr;
   uint8_t buff[4];
   size_t source_len = sizeof(GSL2038_FW) / sizeof(struct fw_data);

   for (size_t source_line = 0; source_line < source_len; source_line++)
   {
      addr = GSL2038_FW[source_line].offset;
      buff[0] = (char) (GSL2038_FW[source_line].val & 0x000000ff);
      buff[1] = (char) ((GSL2038_FW[source_line].val & 0x0000ff00) >> 8);
      buff[2] = (char) ((GSL2038_FW[source_line].val & 0x00ff0000) >> 16);
      buff[3] = (char) ((GSL2038_FW[source_line].val & 0xff000000) >> 24);
      //ESP_LOGI(TAG,"nb %d",source_line);
      GSL2038TransmitData(addr, buff, 4);
   }
}

void IRAM_ATTR GSL2038_isr_handler(void *para)
{
   xSemaphoreGiveFromISR(QueueTouch, 0);
}

void GSL2038Init()
{
   ESP_LOGI(TAG, "GSL2038: Start boot up sequence");

   QueueTouch = xSemaphoreCreateBinary();//xQueueCreate(3,sizeof(TouchPointType));
   xSemaphoreTake(QueueTouch, 0);

   gpio_config_t io_conf;
   //interrupt of rising edge
   io_conf.intr_type = GPIO_INTR_DISABLE;
   //bit mask of the pins
   io_conf.pin_bit_mask = (1ULL << TOUCH_RST);
   //set as input mode
   io_conf.mode = GPIO_MODE_OUTPUT;
   //enable pull-up mode
   io_conf.pull_up_en = 0;
   io_conf.pull_down_en = 0;
   gpio_config(&io_conf);

   gpio_set_level(TOUCH_RST, 0);
   vTaskDelay(50);
   gpio_set_level(TOUCH_RST, 1);
   vTaskDelay(50);

   io_conf.intr_type = GPIO_INTR_NEGEDGE;
   //bit mask of the pins
   io_conf.pin_bit_mask = (1UL << TOUCH_IRQ);
   //set as input mode
   io_conf.mode = GPIO_MODE_INPUT;
   //enable pull-up mode
   io_conf.pull_up_en = 1;
   gpio_config(&io_conf);

   gpio_install_isr_service(0);
   //hook isr handler for specific gpio pin
   gpio_isr_handler_add(TOUCH_IRQ, GSL2038_isr_handler, NULL);

   i2c_config_t conf = {
         .mode = I2C_MODE_MASTER,
         .sda_io_num = TOUCH_SDA,
         .scl_io_num = TOUCH_SCL,
         .sda_pullup_en = GPIO_PULLUP_ENABLE,
         .scl_pullup_en = GPIO_PULLUP_ENABLE,
         .master.clk_speed = 400000,
         .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
   };
   i2c_param_config(IICPortNumber, &conf);

   i2c_driver_install(IICPortNumber, conf.mode, 0, 0, 0);

   // CTP startup sequence
   GSL2038Reset();
   ESP_LOGI(TAG, "GSL2038: Load FW");
   GSL2038LoadFirmware();
   GSL2038Reset();
   GSL2038Start();
   ESP_LOGI(TAG, "GSL2038: Boot up complete");
}

bool GSL2308GetPoint(uint16_t *x, uint16_t *y)
{
   if (xSemaphoreTake(QueueTouch, 0) == pdTRUE)
   {
      TouchPointType point[5];
      int NBFinger = GSL2038ReadTouchPoint(point);
      if (NBFinger > 0)
      {
         *x = point[0].X;
         *y = point[0].Y;
         //ESP_LOGI(TAG, "Touch x:%d y:%d", *x, *y);
         return true;
      }
   }
   return false;
}

void getCord()
{
   if (xSemaphoreTake(QueueTouch, 1) == pdTRUE)
   {
      TouchPointType point[5];
      int NBFinger = GSL2038ReadTouchPoint(point);
      if (NBFinger > 0)
      {
         ESP_LOGI(TAG, "Touch x:%f y:%d",point[0].X/3.2, point[0].Y/2-25);
      }
   }
}