/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "i2s_example_pins.h"

/* Set 1 to allocate rx & tx channels in duplex mode on a same I2S controller, they will share the BCLK and WS signal
 * Set 0 to allocate rx & tx channels in simplex mode, these two channels will be totally separated,
 * Specifically, due to the hardware limitation, the simplex rx & tx channels can't be registered on the same controllers on ESP32 and ESP32-S2,
 * and ESP32-S2 has only one I2S controller, so it can't allocate two simplex channels */
#define EXAMPLE_I2S_DUPLEX_MODE         CONFIG_USE_DUPLEX




#define EXAMPLE_STD_BCLK_IO2        EXAMPLE_I2S_BCLK_IO2     // I2S bit clock io number
#define EXAMPLE_STD_WS_IO2          EXAMPLE_I2S_WS_IO2     // I2S word select io number
#define EXAMPLE_STD_DOUT_IO2        EXAMPLE_I2S_DOUT_IO2     // I2S data out io number
#define EXAMPLE_STD_DIN_IO2         EXAMPLE_I2S_DIN_IO2     // I2S data in io number

#define SAMPLING_RATE 44100
#define SAMPLE_COUNT_32 512
#define EXAMPLE_BUFF_SIZE               sizeof(int32_t) * SAMPLE_COUNT_32

static i2s_chan_handle_t                tx_chan;        // I2S tx channel handler
static i2s_chan_handle_t                rx_chan;        // I2S rx channel handler



static void i2s_example_init_std_simplex(void)
{
    /* Setp 1: Determine the I2S channel configuration and allocate two channels one by one
     * The default configuration can be generated by the helper macro,
     * it only requires the I2S controller id and I2S role
     * The tx and rx channels here are registered on different I2S controller,
     * Except ESP32 and ESP32-S2, others allow to register two separate tx & rx channels on a same controller */

    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    /* Step 2: Setting the configurations of standard mode and initialize each channels one by one
     * The slot configuration and clock configuration can be generated by the macros
     * These two helper macros is defined in 'i2s_std.h' which can only be used in STD mode.
     * They can help to specify the slot and clock configurations for initialization or re-configuring */

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLING_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
            .bclk = EXAMPLE_STD_BCLK_IO2,
            .ws   = EXAMPLE_STD_WS_IO2,
            .dout = EXAMPLE_STD_DOUT_IO2,
            .din  = EXAMPLE_STD_DIN_IO2,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    /* Default is only receiving left slot in mono mode,
     * update to right here to show how to change the default configuration */
     // for left channel tie the L/R pin low (default) for right channel pull L/R to VDD
    rx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));
}

uint8_t *r_buf;
size_t r_bytes;

int32_t raw_samples[SAMPLE_COUNT_32];
void loop() {
    /* ATTENTION: The print and delay in the read task only for monitoring the data by human,
     * Normally there shouldn't be any delays to ensure a short polling time,
     * Otherwise the dma buffer will overflow and lead to the data lost */

#if 1
  /* Read i2s data */
  if (i2s_channel_read(rx_chan, r_buf, EXAMPLE_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) {
      Serial.printf("Read Task: i2s read %d bytes\n----0----1----2----3----------------------", r_bytes);
      for (int n=0; n < EXAMPLE_BUFF_SIZE; n+= 16) {
        Serial.printf("\n%4d ",n);
        for (int i=n; i < n+16; i+=4){
            Serial.printf("%02x%02x%02x%02x ",
                r_buf[i], r_buf[i+1], r_buf[i+2], r_buf[i+3]);
        }
      }
  } else {
      Serial.printf("Read Task: i2s read failed\n");
  }
#else
  if (i2s_channel_read(rx_chan, raw_samples, EXAMPLE_BUFF_SIZE, &r_bytes, 1000) == ESP_OK) {
    int samples_read = r_bytes / sizeof(int32_t);
    // dump the samples out to the serial channel.
    for (int i = 0; i < samples_read; i++)
    {
      Serial.printf("%ld\n", raw_samples[i]);
    }
  } else {
      Serial.printf("Read Task: i2s read failed\n");
  }
#endif
  vTaskDelay(pdMS_TO_TICKS(200));
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  i2s_example_init_std_simplex();
    r_buf = (uint8_t *)calloc(1, EXAMPLE_BUFF_SIZE);
    assert(r_buf); // Check if r_buf allocation success
    r_bytes = 0;

    /* Enable the RX channel */
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
}

