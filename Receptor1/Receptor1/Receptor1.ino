/*
 * RECEPTOR 1 - Demodulador FSK con PARLANTE
 * Proyecto: Sistema de Comunicación Local - CE 1110
 * 
 * Hardware:
 * - Pin GPIO34: Entrada digital FSK (onda cuadrada 0/1)
 * - Pin GPIO26 (DAC2): Salida a parlante/amplificador
 */

#include "arduinoFFT.h"

// ==================== CONFIGURACIÓN ====================
#define SAMPLES 512
#define SAMPLING_FREQUENCY 4000
#define FSK_INPUT_PIN 34

#define F1 800
#define F2 1600

#define UMBRAL_MINIMO     15.0
#define DIFERENCIA_MINIMA 15.0

#define SYNC_PIN 27

// ==================== VARIABLES GLOBALES ====================
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

unsigned long samplingPeriod;
unsigned long contadorAnalisis = 0;

uint8_t bitsRecibidos[8];
int bitCount = 0;

int  lastSync    = LOW;
bool leyendoChar = false;   // estamos recibiendo bits de este carácter
bool syncArmed   = false;   // vimos SYNC↑, estamos esperando el 1er bit válido

// ==================== PROTOTIPOS ====================
void capturarDesdeGPIO();
void aplicarFFT();
void obtenerMagnitudes(double &mag_f1, double &mag_f2);
int  analizarYDemodular();
void mostrarAnalisis(int bit);
void acumularBit(int bit);
char bitsToChar();

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    
    pinMode(FSK_INPUT_PIN, INPUT);
    pinMode(SYNC_PIN, INPUT);
    
    samplingPeriod = round(1000000.0 / SAMPLING_FREQUENCY);
    
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  RECEPTOR 1 - SIN delay()                     ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
    
    Serial.println("✅ Esperando señal FSK...\n");
}

// ==================== LOOP ====================
void loop() {
    static int sinSenalCount = 0;
    
    // 1) Capturar ventana de muestras del GPIO FSK
    capturarDesdeGPIO();
    
    // 2) FFT
    aplicarFFT();
    
    // 3) Demodular bit (0, 1, o -1 si no confiable, modo “estricto”)
    int bit = analizarYDemodular();
    
    // 4) Leer estado actual del pin de handshake
    int sync = digitalRead(SYNC_PIN);

    // Flanco de subida SYNC: LOW -> HIGH
    if (sync == HIGH && lastSync == LOW) {
        // Vimos el comienzo de un carácter, pero NO empezamos a contar todavía.
        syncArmed     = true;   // “listo para arrancar cuando haya 1er bit válido”
        leyendoChar   = false;
        bitCount      = 0;
        sinSenalCount = 0;
        Serial.println("📡 [SYNC_PIN ↑] SYNC detectado, esperando 1er bit válido...");
    }

    lastSync = sync;

    // 5) Mostrar info de debug (usa magnitudes internas)
    mostrarAnalisis(bit);

    // 5.5) Intento de rescate suave si estamos dentro del carácter o justo después del SYNC
    if (bit == -1 && (leyendoChar || syncArmed)) {
        double mag_f1, mag_f2;
        obtenerMagnitudes(mag_f1, mag_f2);
        double maxMag = max(mag_f1, mag_f2);

        // Si hay energía más o menos decente, forzamos decisión
        if (maxMag >= (UMBRAL_MINIMO * 0.7)) {   // 70% del umbral “normal”
            bit = (mag_f2 > mag_f1) ? 1 : 0;
            Serial.println("⚠️  Bit rescatado en modo suave");
        }
    }

    
    // 6) Lógica de silencio / bits
    if (bit == -1) {
        // No pudimos rescatar o no aplica rescate
        // Solo contamos silencio si NO estamos dentro de un carácter
        // ni esperando el primer bit válido
        if (!leyendoChar && !syncArmed) {
            sinSenalCount++;
            if (sinSenalCount >= 2) {
                bitCount = 0;
            }
        }
    } else {
        // bit = 0 o 1 (normal o rescatado)
        sinSenalCount = 0;

        // Si SYNC ya fue detectado y este es el PRIMER bit válido del carácter:
        if (syncArmed) {
            leyendoChar = true;
            bitCount    = 0;   // este bit va a ser el bit 0 del buffer
            syncArmed   = false;
            Serial.println("▶️  1er bit válido del carácter, empezando acumulación");
        }

        // Solo acumulamos si estamos “dentro” del carácter
        if (leyendoChar) {
            acumularBit(bit);
        }
    }
    
    contadorAnalisis++;
}

// ==================== FUNCIONES ====================

void capturarDesdeGPIO() {
    unsigned long microseconds;
    for (int i = 0; i < SAMPLES; i++) {
        microseconds = micros();

        int level = digitalRead(FSK_INPUT_PIN);  // 0 ó 1 desde el cable

        vReal[i] = (double)level - 0.5;      // 0 -> -0.5, 1 -> +0.5
        vImag[i] = 0;

        while (micros() - microseconds < samplingPeriod) {
            // espera activa
        }
    }
}

void aplicarFFT() {
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}

void obtenerMagnitudes(double &mag_f1, double &mag_f2) {
    int index_f1 = round(F1 * SAMPLES / SAMPLING_FREQUENCY);
    int index_f2 = round(F2 * SAMPLES / SAMPLING_FREQUENCY);
    
    mag_f1 = (vReal[index_f1-1] + vReal[index_f1] + vReal[index_f1+1]) / 3.0;
    mag_f2 = (vReal[index_f2-1] + vReal[index_f2] + vReal[index_f2+1]) / 3.0;
}

int analizarYDemodular() {
    double mag_f1, mag_f2;
    obtenerMagnitudes(mag_f1, mag_f2);
    
    double maxMag     = max(mag_f1, mag_f2);
    double diferencia = abs(mag_f1 - mag_f2);
    
    if (maxMag < UMBRAL_MINIMO || diferencia < DIFERENCIA_MINIMA) {
        return -1;
    }
    
    return (mag_f2 > mag_f1) ? 1 : 0;
}

void mostrarAnalisis(int bit) {
    double mag_f1, mag_f2;
    obtenerMagnitudes(mag_f1, mag_f2);
    
    Serial.print("Análisis #");
    Serial.print(contadorAnalisis);
    Serial.print(" | Mag@800: ");
    Serial.print(mag_f1, 0);
    Serial.print(" | Mag@1600: ");
    Serial.print(mag_f2, 0);
    Serial.print(" | Bit: ");
    
    if (bit == -1) {
        Serial.print("❌");
    } else {
        Serial.print(bit);
    }

    Serial.print(" | leyendoChar=");
    Serial.print(leyendoChar ? "1" : "0");
    Serial.print(" | syncArmed=");
    Serial.print(syncArmed ? "1" : "0");
    Serial.print(" | Bits parciales: [");
    for (int i = 0; i < bitCount; i++) {
        Serial.print(bitsRecibidos[i]);
    }
    Serial.println("]");
}

void acumularBit(int bit) {
    if (bitCount < 8) {
        bitsRecibidos[bitCount] = bit;
        bitCount++;
    }
    
    if (bitCount >= 8) {
        char c = bitsToChar();

        // PREÁMBULO opcional si lo usás
        if (c == 0x55 || c == 'U') {
            Serial.println("📡 [PREÁMBULO detectado - descartando]");
            bitCount    = 0;
            leyendoChar = false;  // esperar próximo SYNC
            return;
        }

        Serial.print("\n🔊 CARÁCTER: '");
        Serial.print(c);
        Serial.print("' (ASCII ");
        Serial.print((int)c);
        Serial.println(")\n");
        
        bitCount    = 0;
        leyendoChar = false;  // fin de este carácter, esperar nuevo SYNC↑
    }
}

char bitsToChar() {
    int ascii = 0;
    for (int i = 0; i < 8; i++) {
        ascii = (ascii << 1) | bitsRecibidos[i];
    }
    return (char)ascii;
}
