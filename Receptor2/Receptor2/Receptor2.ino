/*
 * RECEPTOR 2 - Demodulador FSK (canal ALTO) con parlante
 * Proyecto: Sistema de Comunicación Local - CE 1110
 *
 * Hardware:
 * - Pin GPIO34: Entrada digital FSK (onda cuadrada 0/1)
 * - DAC GPIO26: salida de audio (ESP32)
 *
 * Notas:
 * - Este receptor queda anclado a F1=1400 Hz (bit 0) y F2=1700 Hz (bit 1),
 *   para separar totalmente del canal BAJO (p. ej., 360/540 Hz).
 */

#include "arduinoFFT.h"

// ==================== CONFIGURACIÓN ====================
#define SAMPLES 512
#define SAMPLING_FREQUENCY 4000
#define FSK_INPUT_PIN 34
#define AUDIO_OUTPUT_PIN 26

// Canal ALTO (ASCII/seno o FSK alto)
#define F1 1400     // bit 0
#define F2 1700     // bit 1

// Frecuencias a reproducir en parlante según bit detectado (libres a elección)
#define FREQ_AUDIO_GRAVE 700    // Hz cuando se detecta F1 (1400 Hz)
#define FREQ_AUDIO_AGUDA 1200   // Hz cuando se detecta F2 (1700 Hz)

#define UMBRAL_MINIMO 20.0
#define DIFERENCIA_MINIMA 15.0

// ==================== VARIABLES GLOBALES ====================
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

unsigned long samplingPeriod;
unsigned long contadorAnalisis = 0;

// Audio sin delay
bool reproduciendo = false;
int  frecuenciaAudio = 0;
unsigned long tiempoAnteriorAudio = 0;
const int FS_AUDIO = 8000;
const int PERIODO_AUDIO_US = 1000000 / FS_AUDIO;  // 125 μs

// Silencio: cuántas ventanas sin tono para apagar audio
int sinSenalCount = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(FSK_INPUT_PIN, INPUT);
  samplingPeriod = round(1000000.0 / SAMPLING_FREQUENCY);

  // Centrar DAC en silencio
  dacWrite(AUDIO_OUTPUT_PIN, 128);

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║  RECEPTOR 2 - AUDIO FSK (ALTO: 1400/1700 Hz)  ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");
  Serial.print("Fs = "); Serial.print(SAMPLING_FREQUENCY);
  Serial.print(" Hz, N = "); Serial.println(SAMPLES);
  Serial.print("Δf ≈ "); Serial.print((double)SAMPLING_FREQUENCY / SAMPLES, 3);
  Serial.println(" Hz\n");
  Serial.println("✅ Esperando señal FSK (1400/1700 Hz)...");
}

// ==================== LOOP ====================
void loop() {
  if (reproduciendo) actualizarAudio();

  // Capturar y procesar
  capturarDesdeGPIO();
  aplicarFFT();

  int bit = analizarYDemodular();
  mostrarAnalisis(bit);

  if (bit == -1) {
    sinSenalCount++;
    if (sinSenalCount >= 3) {
      reproduciendo = false;
      frecuenciaAudio = 0;
      dacWrite(AUDIO_OUTPUT_PIN, 128);  // silencio (nivel medio)
    }
  } else {
    sinSenalCount = 0;
    frecuenciaAudio = (bit == 0) ? FREQ_AUDIO_GRAVE : FREQ_AUDIO_AGUDA;
    reproduciendo = true;
  }

  contadorAnalisis++;
}

// ==================== FUNCIONES ====================

void capturarDesdeGPIO() {
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long t0 = micros();

    int level = digitalRead(FSK_INPUT_PIN);   // 0/1
    vReal[i] = (double)level - 0.5;           // centra en -0.5/+0.5
    vImag[i] = 0.0;

    while (micros() - t0 < samplingPeriod) { /* busy-wait */ }
  }
}

void aplicarFFT() {
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
}

int analizarYDemodular() {
  int index_f1 = round((double)F1 * SAMPLES / SAMPLING_FREQUENCY);
  int index_f2 = round((double)F2 * SAMPLES / SAMPLING_FREQUENCY);

  // límites seguros para usar [k-1,k,k+1]
  index_f1 = constrain(index_f1, 1, SAMPLES/2 - 2);
  index_f2 = constrain(index_f2, 1, SAMPLES/2 - 2);

  double mag_f1 = (vReal[index_f1-1] + vReal[index_f1] + vReal[index_f1+1]) / 3.0;
  double mag_f2 = (vReal[index_f2-1] + vReal[index_f2] + vReal[index_f2+1]) / 3.0;

  double maxMag     = (mag_f1 > mag_f2) ? mag_f1 : mag_f2;
  double diferencia = fabs(mag_f1 - mag_f2);

  if (maxMag < UMBRAL_MINIMO || diferencia < DIFERENCIA_MINIMA) return -1;
  return (mag_f2 > mag_f1) ? 1 : 0;
}

void mostrarAnalisis(int bit) {
  int index_f1 = round((double)F1 * SAMPLES / SAMPLING_FREQUENCY);
  int index_f2 = round((double)F2 * SAMPLES / SAMPLING_FREQUENCY);

  index_f1 = constrain(index_f1, 1, SAMPLES/2 - 2);
  index_f2 = constrain(index_f2, 1, SAMPLES/2 - 2);

  double mag_f1 = (vReal[index_f1-1] + vReal[index_f1] + vReal[index_f1+1]) / 3.0;
  double mag_f2 = (vReal[index_f2-1] + vReal[index_f2] + vReal[index_f2+1]) / 3.0;

  Serial.print("Análisis #"); Serial.print(contadorAnalisis);
  Serial.print(" | Mag@"); Serial.print(F1); Serial.print(": ");
  Serial.print(mag_f1, 0);
  Serial.print(" | Mag@"); Serial.print(F2); Serial.print(": ");
  Serial.print(mag_f2, 0);
  Serial.print(" | Bit: ");
  if (bit == -1) Serial.println("❌");
  else           Serial.println(bit);
}

void actualizarAudio() {
  if (frecuenciaAudio <= 0) return;

  unsigned long t = micros();
  if (t - tiempoAnteriorAudio >= PERIODO_AUDIO_US) {
    tiempoAnteriorAudio = t;

    static unsigned long faseContador = 0;
    faseContador++;
    float tt = (float)faseContador / FS_AUDIO;

    int valor = 128 + 100 * sin(2 * PI * frecuenciaAudio * tt);
    dacWrite(AUDIO_OUTPUT_PIN, constrain(valor, 0, 255));
  }
}
