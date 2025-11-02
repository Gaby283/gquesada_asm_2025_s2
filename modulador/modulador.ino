#include <arduinoFFT.h>
#include <Arduino.h>
#include <math.h>

// ---------- PINES ----------
const int OUT1_PIN = 25;  // señal 1
const int OUT2_PIN = 26;  // señal 2
const int ADC_PIN  = 34;  // entrada ADC (si deseas mantener FFT)

// ---------- FRECUENCIAS ----------
const int f1_low  = 100;   // señal 1: bit = 0
const int f1_high = 200;   // señal 1: bit = 1
const int f2_low  = 300;   // señal 2: bit = 0
const int f2_high = 400;   // señal 2: bit = 1

// ---------- Variables de modulación ----------
volatile int bit1 = 0;
volatile int bit2 = 0;
volatile int f_mod1 = 100;
volatile int f_mod2 = 300;

// ---------- Generadores ----------
volatile unsigned long halfPeriod1 = 500;
volatile unsigned long halfPeriod2 = 500;
volatile bool state1 = false;
volatile bool state2 = false;

// ---------- Control de cambio de bits ----------
const unsigned long changeIntervalMs = 2000;
unsigned long lastChangeMs = 0;

// ---------- FFT ----------
const uint16_t SAMPLES = 256;
const double SAMPLING_FREQ = 8192.0;
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT FFT = ArduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// ---------- PROTOTIPOS ----------
void setHalfPeriod1(int f);
void setHalfPeriod2(int f);
void updateFreqs();
void generatorTask1(void *pv);
void generatorTask2(void *pv);

void setup() {
  Serial.begin(115200);
  pinMode(OUT1_PIN, OUTPUT);
  pinMode(OUT2_PIN, OUTPUT);
  digitalWrite(OUT1_PIN, LOW);
  digitalWrite(OUT2_PIN, LOW);

  updateFreqs();

  xTaskCreatePinnedToCore(generatorTask1, "GEN1", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(generatorTask2, "GEN2", 2048, NULL, 2, NULL, 1);

  Serial.println("---- MODULADOR 2x2-FSK iniciado ----");
}

void loop() {
  unsigned long now = millis();
  if (now - lastChangeMs >= changeIntervalMs) {
    lastChangeMs = now;
    // Cambiar bits de ambas señales
    bit1 = !bit1;
    bit2 = !bit2;
    updateFreqs();
    Serial.print("[Cambio] bit1="); Serial.print(bit1);
    Serial.print(" bit2="); Serial.print(bit2);
    Serial.print(" f1="); Serial.print(f_mod1);
    Serial.print("Hz f2="); Serial.println(f_mod2);
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

// -------------------------------------------------------

void updateFreqs() {
  f_mod1 = (bit1 == 0) ? f1_low : f1_high;
  f_mod2 = (bit2 == 0) ? f2_low : f2_high;
  setHalfPeriod1(f_mod1);
  setHalfPeriod2(f_mod2);
}

void setHalfPeriod1(int f) {
  unsigned long half = 500000UL / (unsigned long)f;
  if (half < 1) half = 1;
  halfPeriod1 = half;
}

void setHalfPeriod2(int f) {
  unsigned long half = 500000UL / (unsigned long)f;
  if (half < 1) half = 1;
  halfPeriod2 = half;
}

// -------------------------------------------------------

void generatorTask1(void *pv) {
  (void) pv;
  unsigned long lastToggle = micros();
  while (true) {
    unsigned long now = micros();
    if ((now - lastToggle) >= halfPeriod1) {
      state1 = !state1;
      digitalWrite(OUT1_PIN, state1 ? HIGH : LOW);
      lastToggle = now;
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void generatorTask2(void *pv) {
  (void) pv;
  unsigned long lastToggle = micros();
  while (true) {
    unsigned long now = micros();
    if ((now - lastToggle) >= halfPeriod2) {
      state2 = !state2;
      digitalWrite(OUT2_PIN, state2 ? HIGH : LOW);
      lastToggle = now;
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
