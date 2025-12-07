// INMP441 -> ESP32 (Arduino IDE)
// Wiring default pins:
//    INMP441 VDD -> 3.3V
//    INMP441 GND -> GND
//    INMP441 L/R -> GND (left channel)
//    INMP441 WS  -> GPIO 25
//    INMP441 SCK -> GPIO 26
//    INMP441 SD  -> GPIO 22

#include <Arduino.h>
#include "driver/i2s.h"

#define I2S_NUM           I2S_NUM_0
#define SAMPLE_RATE       16000            // 8k/16k/44100 are possible; try 16000 first
#define I2S_READ_LEN      1024             // number of 32-bit words to read per block

// Pin assignment (change if you prefer other GPIOs)
const int I2S_WS_PIN  = 25; // LRCLK / WS
const int I2S_SCK_PIN = 26; // BCLK / SCK
const int I2S_SD_PIN  = 22; // DATA (microphone --> ESP32)

void setupI2S() {
  // i2s config
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // read into 32-bit containers
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // INMP441 is mono (tie L/R pin)
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false
  };

  // install and start i2s driver
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);

  // pin config
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };

  i2s_set_pin(I2S_NUM, &pin_config);
  // Set clock if needed (not always required)
  // i2s_set_clk(I2S_NUM, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_FMT_ONLY_LEFT);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("INMP441 I2S mic test - starting");
  setupI2S();
}

void loop() {
  const int bufWords = I2S_READ_LEN;
  int32_t i2s_read_buf[bufWords];
  size_t bytes_read = 0;

  // read raw data (32-bit words). Blocking until buffer fills
  esp_err_t r = i2s_read(I2S_NUM, (void*)i2s_read_buf, bufWords * sizeof(int32_t), &bytes_read, portMAX_DELAY);
  if (r != ESP_OK) {
    Serial.print("i2s_read err: ");
    Serial.println(r);
    delay(200);
    return;
  }
  int samples = bytes_read / 4; // 4 bytes per 32-bit sample

  // compute peak and RMS. INMP441 sends 24-bit left-aligned in a 32-bit word,
  // so shift right by 8 to get 24-bit signed value into lower bits.
  long long sumSq = 0;
  int32_t peak = 0;
  for (int i = 0; i < samples; ++i) {
    int32_t raw = i2s_read_buf[i];
    int32_t s24 = raw >> 8;        // drop the lowest (padding) byte
    int32_t sample = s24;          // signed 24-bit within a 32-bit int

    long long v = sample;
    sumSq += v * v;
    if (abs(sample) > peak) peak = abs(sample);
  }

  double rms = sqrt((double)sumSq / (double)samples);
  // Normalize RMS/peak to [-1..1] approx: 24-bit max ~ 2^23
  const double max24 = (double)(1 << 23);
  double rms_norm = rms / max24;
  double peak_norm = (double)peak / max24;

  // Convert to dBFS (approx)
  double rms_db = 20.0 * log10(max(1e-12, rms_norm));
  double peak_db = 20.0 * log10(max(1e-12, peak_norm));

  Serial.print("Samples: ");
  Serial.print(samples);
  Serial.print("  RMS: ");
  Serial.print(rms_norm, 6);
  Serial.print(" (");
  Serial.print(rms_db, 2);
  Serial.print(" dBFS)  Peak: ");
  Serial.print(peak_norm, 6);
  Serial.print(" (");
  Serial.print(peak_db, 2);
  Serial.println(" dBFS)");

  // small delay to allow Serial output to catch up
  delay(50);
}