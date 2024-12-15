#include <driver/i2s.h>
#include <arduinoFFT.h>

#include <SSD1306Wire.h>  // v4.6.1  specific version for esp32 https://github.com/ThingPulse/esp8266-oled-ssd1306


// you shouldn't need to change these settings
#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_RATE 8000
// most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
// either wire your microphone to the same pins or change these to match your wiring
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_26
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_22
#define I2S_MIC_SERIAL_DATA GPIO_NUM_21


// HelTec ESP32 DevKit parameters
#define D3 4
#define D5 15
SSD1306Wire  oled(0x3c, D3, D5);


// don't mess around with this
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// and don't mess around with this
i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};


float vReal[SAMPLE_BUFFER_SIZE];
float vImag[SAMPLE_BUFFER_SIZE];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

typedef union {
  float f;
  struct {
    unsigned int mantisa : 23;
    unsigned int exponent : 8;
    unsigned int sign : 1;
  } parts;
} float_cast;

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLE_BUFFER_SIZE, SAMPLE_RATE);

void oledSetup(void) {
  // reset OLED
  pinMode(16,OUTPUT); 
  digitalWrite(16,LOW); 
  delay(50); 
  digitalWrite(16,HIGH); 
  
  oled.init();
  oled.clear();
  oled.flipScreenVertically();
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(0 , 0, "START" );
  oled.display();
}

void setup()
{
  // we need serial output for the plotter
  Serial.begin(115200);
  // start up the I2S peripheral
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);
  oledSetup();
}

int32_t raw_samples[SAMPLE_BUFFER_SIZE];
void loop()
{
  // read from the I2S device
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, raw_samples, sizeof(int32_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int32_t);
  float abs_moy = 0.0;
  float moy = 0.0;
  for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++){
    vReal[i] = raw_samples[i];
    abs_moy += abs(vReal[i]);
    moy += vReal[i];
    vImag[i]=0.0;
  }
  abs_moy = abs_moy / SAMPLE_BUFFER_SIZE;
  moy = moy / SAMPLE_BUFFER_SIZE;
  for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++){
    vReal[i] -= moy;
  }
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	/* Weigh data */
  float moy2 = 0.0;
  for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++){
    moy2 += vReal[i];
  }
  for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++){
    vReal[i] -= moy2;
  }
  FFT.compute(FFTDirection::Forward); /* Compute FFT */
  FFT.complexToMagnitude(); /* Compute magnitudes */

  float x = FFT.majorPeakParabola();
  //Serial.printf("abs_moy: %6.2f moy: %6.2f moy2: %6.2f f peak: %5.0f\n", abs_moy, moy, moy2, x);

  // float_cast d1, d2;
  // d1.f = 1.0f;
  // d2.f = log10f_fast(abs_moy);
  // Serial.printf("sign = %x exponent = %3d mantisa = %x  abs_moy = %6.2f sign = %x exponent = %3d mantisa = %x\n", 
  //             d1.parts.sign, d1.parts.exponent, d1.parts.mantisa, abs_moy,
  //             d2.parts.sign, d2.parts.exponent, d2.parts.mantisa);
  float db_value = 50 + log10f_fast(abs_moy) *10;
  Serial.printf("abs_moy: %6.2f log10(abs_moy): %6.2f\n", abs_moy, db_value);

// dump the samples out to the serial channel.
  float max = 0;
  for (int i = 0; i < samples_read; i++)
  {
    if (vReal[i] > max){
      max = vReal[i];
    }
  }
  oled.clear();
  for (int i = 20; i < samples_read; i+=2)
  {
    float m = 0;
    float vReal_value = (vReal[i]+vReal[i+1])/2;
    float vReal_db_value = -60 + log10f_fast(vReal_value) *10; // in db, needs offset for  display
    oled.setPixelColor(i/2, 54 - vReal_db_value, WHITE);
  }
  oled.drawProgressBar(0 , 60, 128, 4, db_value);
  oled.display();
  // if (moy > 1.0){
  //   Serial.printf("max: %5.2f ", max);
  //   Serial.println("Computed magnitudes:");
  //   PrintVector(vReal, (samples_read >> 1), SCL_FREQUENCY);
  //   while(1); /* Run Once */
  // }
}


void PrintVector(float *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    float abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / 8000);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * 8000) / 512);
	break;
    }
    Serial.print(abscissa, 2);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

/**
   Fast algorithm for log10

   This is a fast approximation to log2()
   Y = C[0]*F*F*F + C[1]*F*F + C[2]*F + C[3] + E;
   log10f is exactly log2(x)/log2(10.0f)
   Math_log10f_fast(x) =(log2f_approx(x)*0.3010299956639812f)

   @param X number want log10 for
   @return log10(x)
*/
float log10f_fast(float X) {
  float Y, F;
  int E;
  F = frexpf(fabsf(X), &E);
  Y = 1.23149591368684f;
  Y *= F;
  Y += -4.11852516267426f;
  Y *= F;
  Y += 6.02197014179219f;
  Y *= F;
  Y += -3.13396450166353f;
  Y += E;
  return (Y * 0.3010299956639812f);
}

