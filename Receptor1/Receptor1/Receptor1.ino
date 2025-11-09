/*
 * RECEPTOR FSK – Canal ALTO (1400/1700) con preámbulo y reloj por ventana
 * Proyecto: Sistema de Comunicación Local - CE 1110
 *
 * Requiere TX con: TB = 128 ms, GUARD ≈ 110 ms, preámbulo 0x55 × 2.
 * Estrategia RX:
 *  - Busca 0x55 dos veces (SEARCH_PREAMBLE).
 *  - Salta 1 ventana para alinearse al primer bit útil.
 *  - Lee EXACTAMENTE 8 ventanas (1 ventana = 1 bit) con “hold-last” si hay ❌.
 */

#include "arduinoFFT.h"

// ==================== CONFIGURACIÓN ====================
#define SAMPLES 512
#define SAMPLING_FREQUENCY 4000
#define FSK_INPUT_PIN 34

// Canal ALTO (texto)
#define F1 1400        // bit 0
#define F2 1700        // bit 1

// Umbrales afinados
#define UMBRAL_MINIMO      20.0
#define DIFERENCIA_MINIMA  28.0

// ==================== VARIABLES GLOBALES ====================
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

unsigned long samplingPeriod;
unsigned long contadorAnalisis = 0;

// Estado de decodificación con preámbulo
enum RxState { SEARCH_PREAMBLE, READ_DATA };
RxState rxState = SEARCH_PREAMBLE;
uint8_t preCount = 0;       // cuántas veces seguidas vimos 0x55
uint8_t dataBits[8];        // buffer de payload
int     dataIdx = 0;        // índice 0..7

// Control de silencio
int sinSenalCount = 0;

// Reloj por ventana / robustez
int  framesPerBit = 1;      // 1 ventana FFT = 1 bit (TB=128 ms)
bool haveLastDecision = false;
int  lastDecision = 0;
bool skipOneWindow = false;  // al pasar de preámbulo a datos

// ==================== PROTOTIPOS ====================
void capturarDesdeGPIO();
void aplicarFFT();
int  analizarYDemodular();
void mostrarAnalisis(int bit);

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(FSK_INPUT_PIN, INPUT);
  samplingPeriod = round(1000000.0 / SAMPLING_FREQUENCY);

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║  RECEPTOR FSK - ALTO (1400/1700) c/ preámbulo  ║");
  Serial.println("║  1 ventana = 1 bit, hold-last, skipOneWindow   ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");

  Serial.print("Fs = "); Serial.print(SAMPLING_FREQUENCY);
  Serial.print(" Hz, N = "); Serial.println(SAMPLES);
  Serial.print("Δf ≈ "); Serial.print((double)SAMPLING_FREQUENCY / SAMPLES, 3);
  Serial.println(" Hz\n");

  Serial.println("✅ Buscando 0x55 × 2; luego 8 bits de payload…");
}

// ==================== LOOP ====================
void loop() {
  // 1) Captura + FFT
  capturarDesdeGPIO();
  aplicarFFT();

  // 2) Demodulación binaria (bit = 0/1; -1 si sin decisión)
  int bit = analizarYDemodular();
  mostrarAnalisis(bit);

  // 3) Máquina de estados de decodificación
  if (bit == -1) {
    // ❌ no avances estado
    sinSenalCount++;
    if (sinSenalCount >= 3) { // ~3*128 ms sin señal
      rxState = SEARCH_PREAMBLE;
      dataIdx = 0;
      preCount = 0;
      sinSenalCount = 0;
      haveLastDecision = false;
      skipOneWindow = false;
    }
  } else {
    sinSenalCount = 0;

    if (rxState == SEARCH_PREAMBLE) {
      // Shift-register de 8 bits para detectar 0x55 (01010101)
      static uint8_t shift = 0;
      shift = ((shift << 1) | (bit & 1)) & 0xFF;

      if (shift == 0x55) {
        preCount++;
        if (preCount >= 2) {
          // Lock logrado → alinear y pasar a datos
          rxState = READ_DATA;
          dataIdx = 0;
          preCount = 0;
          haveLastDecision = false;
          skipOneWindow = true;   // saltar 1 ventana para caer al inicio del primer bit útil
        }
      }
      // si no coincide, seguimos buscando

    } else { // READ_DATA
      // Alinear una vez tras el preámbulo
      if (skipOneWindow) {
        skipOneWindow = false; // consumir esta ventana y no acumular
      } else {
        // Decisión robusta para esta ventana
        int cur;
        if (bit == -1) {
          // ventana gris: usa última decisión si existe, si no, 0 neutro
          cur = haveLastDecision ? lastDecision : 0;
        } else {
          cur = bit;
          lastDecision = bit;
          haveLastDecision = true;
        }

        // 1 ventana = 1 bit
        dataBits[dataIdx++] = cur;

        if (dataIdx >= 8) {
          // Convertir 8 bits MSB-first
          uint8_t val = 0;
          for (int i = 0; i < 8; i++)
            val = (val << 1) | (dataBits[i] & 1);

          Serial.print("\n🔊 CARÁCTER: '");
          Serial.print((char)val);
          Serial.print("' (ASCII ");
          Serial.print((int)val);
          Serial.println(")\n");

          // Reiniciar para el siguiente paquete
          rxState = SEARCH_PREAMBLE;
          dataIdx = 0;
          haveLastDecision = false;
          skipOneWindow = false;
        }
      }
    }
  }

  contadorAnalisis++;
}

// ==================== FUNCIONES ====================

void capturarDesdeGPIO() {
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long t0 = micros();

    // Lectura digital 0/1 → centrado en -0.5/+0.5 (reduce DC)
    int level = digitalRead(FSK_INPUT_PIN);
    vReal[i] = (double)level - 0.5;   // 0 -> -0.5, 1 -> +0.5
    vImag[i] = 0.0;

    // Mantener Fs exacta
    while (micros() - t0 < samplingPeriod) { /* busy-wait */ }
  }
}

void aplicarFFT() {
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
}

int analizarYDemodular() {
  // Índices FFT para F1 y F2
  int index_f1 = round((double)F1 * SAMPLES / SAMPLING_FREQUENCY);
  int index_f2 = round((double)F2 * SAMPLES / SAMPLING_FREQUENCY);

  // Límites seguros para usar [k-1, k, k+1]
  index_f1 = constrain(index_f1, 1, SAMPLES/2 - 2);
  index_f2 = constrain(index_f2, 1, SAMPLES/2 - 2);

  // Energía promediando 3 bins (mitiga pequeño desajuste de bin)
  double mag_f1 = (vReal[index_f1-1] + vReal[index_f1] + vReal[index_f1+1]) / 3.0;
  double mag_f2 = (vReal[index_f2-1] + vReal[index_f2] + vReal[index_f2+1]) / 3.0;

  double maxMag     = (mag_f1 > mag_f2) ? mag_f1 : mag_f2;
  double diferencia = fabs(mag_f1 - mag_f2);

  // Criterios de validez
  if (maxMag < UMBRAL_MINIMO || diferencia < DIFERENCIA_MINIMA) {
    return -1; // sin decisión clara
  }

  // Bit = 0 si domina F1 (1400), Bit = 1 si domina F2 (1700)
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
  Serial.print(" | Mag@");    Serial.print(F1); Serial.print(": ");
  Serial.print(mag_f1, 0);
  Serial.print(" | Mag@");    Serial.print(F2); Serial.print(": ");
  Serial.print(mag_f2, 0);
  Serial.print(" | Bit: ");
  if (bit == -1) Serial.println("❌"); else Serial.println(bit);
}
