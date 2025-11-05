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
#define SAMPLES 512                  // Muestras para FFT
#define SAMPLING_FREQUENCY 4000      // 4 kHz (ajustado, era 3500)
#define FSK_INPUT_PIN 34             // Pin entrada digital
#define AUDIO_OUTPUT_PIN 26          // Pin DAC salida (parlante)

// Frecuencias FSK
#define F1 800   // Bit 0
#define F2 1600  // Bit 1

// Umbrales
#define UMBRAL_MINIMO 50.0
#define DIFERENCIA_MINIMA 20.0

// ==================== VARIABLES GLOBALES ====================
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

unsigned long samplingPeriod;
unsigned long contadorAnalisis = 0;

uint8_t bitsRecibidos[8];
int bitCount = 0;

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  RECEPTOR 1 - Demodulador FSK (PARLANTE)      ║");
    Serial.println("║  CE 1110 - Análisis de Señales Mixtas         ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
    
    pinMode(FSK_INPUT_PIN, INPUT);
    
    samplingPeriod = round(1000000.0 / SAMPLING_FREQUENCY);
    
    Serial.println("📋 CONFIGURACIÓN:");
    Serial.print("  • Fs: ");
    Serial.print(SAMPLING_FREQUENCY);
    Serial.println(" Hz");
    Serial.print("  • Muestras FFT: ");
    Serial.println(SAMPLES);
    Serial.print("  • Resolución: ");
    Serial.print((double)SAMPLING_FREQUENCY / SAMPLES, 2);
    Serial.println(" Hz");
    Serial.print("  • Pin entrada: GPIO");
    Serial.println(FSK_INPUT_PIN);
    Serial.print("  • Pin parlante: GPIO");
    Serial.println(AUDIO_OUTPUT_PIN);
    
    Serial.println("\n✅ Esperando señal FSK...\n");
    delay(1000);
}

// ==================== LOOP ====================
void loop() {
    capturarDesdeGPIO();
    aplicarFFT();
    
    int bit = analizarYDemodular();
    mostrarAnalisis(bit);
    
    if (bit != -1) {
        acumularBit(bit);
    }
    
    contadorAnalisis++;
}

// ==================== FUNCIONES ====================

void capturarDesdeGPIO() {
    unsigned long microseconds;
    for (int i = 0; i < SAMPLES; i++) {
        microseconds = micros();

        int level = digitalRead(FSK_INPUT_PIN);  // 0 ó 1 desde el cable

        // Opción 1: usar 0 y 1 pero centrado:  -0.5 / +0.5
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

int analizarYDemodular() {
    int index_f1 = round(F1 * SAMPLES / SAMPLING_FREQUENCY);
    int index_f2 = round(F2 * SAMPLES / SAMPLING_FREQUENCY);
    
    double mag_f1 = (vReal[index_f1-1] + vReal[index_f1] + vReal[index_f1+1]) / 3.0;
    double mag_f2 = (vReal[index_f2-1] + vReal[index_f2] + vReal[index_f2+1]) / 3.0;
    
    double maxMag = max(mag_f1, mag_f2);
    double diferencia = abs(mag_f1 - mag_f2);
    
    if (maxMag < UMBRAL_MINIMO || diferencia < DIFERENCIA_MINIMA) {
        return -1;
    }
    
    return (mag_f2 > mag_f1) ? 1 : 0;
}

void mostrarAnalisis(int bit) {
    int index_f1 = round(F1 * SAMPLES / SAMPLING_FREQUENCY);
    int index_f2 = round(F2 * SAMPLES / SAMPLING_FREQUENCY);
    
    double mag_f1 = (vReal[index_f1-1] + vReal[index_f1] + vReal[index_f1+1]) / 3.0;
    double mag_f2 = (vReal[index_f2-1] + vReal[index_f2] + vReal[index_f2+1]) / 3.0;
    
    Serial.print("Análisis #");
    Serial.print(contadorAnalisis);
    Serial.print(" | Mag@800: ");
    Serial.print(mag_f1, 0);
    Serial.print(" | Mag@1600: ");
    Serial.print(mag_f2, 0);
    Serial.print(" | Bit: ");
    
    if (bit == -1) {
        Serial.println("❌");
    } else {
        Serial.print(bit);
        Serial.print(" | Bits: [");
        for (int i = 0; i < bitCount; i++) {
            Serial.print(bitsRecibidos[i]);
        }
        Serial.println("]");
    }
}

void acumularBit(int bit) {
    bitsRecibidos[bitCount] = bit;
    bitCount++;
    
    if (bitCount >= 8) {
        char c = bitsToChar();
        
        Serial.print("\n🔊 CARÁCTER RECIBIDO: '");
        Serial.print(c);
        Serial.print("' (ASCII ");
        Serial.print((int)c);
        Serial.println(")\n");
        
        reproducirCaracter(c);
        
        bitCount = 0;
    }
}

char bitsToChar() {
    int ascii = 0;
    for (int i = 0; i < 8; i++) {
        ascii = (ascii << 1) | bitsRecibidos[i];
    }
    return (char)ascii;
}

void reproducirCaracter(char c) {
    // Tono según carácter (200-2500 Hz)
    int frecuencia = 200 + ((int)c * 10);
    int duracion = 300;  // 300 ms
    
    reproducirTono(frecuencia, duracion);
}

// ⭐ FUNCIÓN CORREGIDA - Usa Fs diferente para reproducción
void reproducirTono(int freq, int duracion_ms) {
    const int FS_AUDIO = 8000;  // Fs para audio del parlante (independiente)
    int muestras = (duracion_ms * FS_AUDIO) / 1000;
    int periodo_us = 1000000 / FS_AUDIO;  // 125 μs @ 8kHz
    
    for (int i = 0; i < muestras; i++) {
        float t = i / (float)FS_AUDIO;
        int valor = 128 + 100 * sin(2 * PI * freq * t);
        dacWrite(AUDIO_OUTPUT_PIN, constrain(valor, 0, 255));
        delayMicroseconds(periodo_us);
    }
    
    // Silencio
    dacWrite(AUDIO_OUTPUT_PIN, 128);
}