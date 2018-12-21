#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA (1013.25)

#define OUTPUTENABLE
#define MAXPWM        127

#define Kp        1.0f
#define Ki        0.0f
#define IMAX      10.0f
#define CYCLETIME 8000000     //cycleTime/0.0000000125 8000000 -> 100ms

#define BMP_SCK 14
#define BMP_MISO 12
#define BMP_MOSI 13
#define BMP_CS 15

#define LEDPIN  2
#define MOTPIN  5

struct reg_t {
  float altitude;
  float altitudeTarget;
  float altitudeOffset;
  float error;
  uint8_t pwm;
} reg;

Adafruit_BMP3XX bmp(BMP_CS);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, LEDPIN, NEO_RGBW + NEO_KHZ800);

// used to calculate next interrupt
volatile unsigned long next;

// regulator
void inline regulator(void) {
  next = next + CYCLETIME; // calculate when the next interrupt should be called
  timer0_write(next);

  reg.altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - reg.altitudeOffset;
  reg.error = reg.altitudeTarget - reg.altitude;


  bmp.performReading();
}

void setup() {
  bmp.begin();  // Initialize the pressure sensor
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  strip.begin();  // Initialize all pixels
  strip.show(); 

#ifdef OUTPUTENABLE
  pinMode(MOTPIN, OUTPUT); // pwm output
  analogWrite(MOTPIN, 0);
#endif

  bmp.performReading();   // set offset from sea level
  delay(50);
  reg.altitudeOffset = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  bmp.performReading();

  timer0_isr_init();    // start regulator timer
  timer0_attachInterrupt(regulator);
  next = ESP.getCycleCount() + CYCLETIME;
  timer0_write(next);
  interrupts();
}

void loop() {
  colorWipe(strip.Color(100, 0, 80), 50); // Red
}

void setPwm(uint8_t _pwm) {
  if (_pwm <= 0) _pwm = 0;
  if (_pwm >= MAXPWM) _pwm = MAXPWM;
#ifdef OUTPUTENABLE
  analogWrite(MOTPIN, _pwm);
#endif
}

void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
}
