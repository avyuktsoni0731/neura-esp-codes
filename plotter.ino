#include <Arduino.h>
#include "driver/i2s.h"

#define I2S_NUM           I2S_NUM_0
#define SAMPLE_RATE       16000
#define I2S_READ_LEN      1024

// Pin configuration (same as your working code)
const int I2S_WS_PIN  = 25;
const int I2S_SCK_PIN = 26;
const int I2S_SD_PIN  = 22;

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };

  i2s_set_pin(I2S_NUM, &pin_config);
}

void setup() {
  Serial.begin(115200);
  setupI2S();
}

void loop() {
  int32_t buffer[I2S_READ_LEN];
  size_t bytesRead = 0;

  // Read I2S data
  i2s_read(I2S_NUM, (void*)buffer, sizeof(buffer), &bytesRead, portMAX_DELAY);
  int samples = bytesRead / 4;

  // Compute peak value
  int32_t peak = 0;

  for (int i = 0; i < samples; i++) {
    int32_t sample = buffer[i] >> 8;  // convert 32-bit to 24-bit
    sample = abs(sample);
    if (sample > peak) peak = sample;
  }

  // Normalize to 0–1 range
  float peakNorm = (float)peak / (float)(1 << 23);

  // Print only ONE value → Serial Plotter friendly
  Serial.println(peakNorm);
}