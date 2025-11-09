/*
 * RECEPTOR 2 - Canal AUDIO (360/540 Hz) con PARLANTE
 */

#include "arduinoFFT.h"

// ==================== CONFIGURACIÓN ====================
#define SAMPLES 512
#define SAMPLING_FREQUENCY 2000  // ← Más bajo (360-540 Hz son bajas)
#define FSK_INPUT_PIN 34
#define AUDIO_OUTPUT_PIN 26  // DAC para parlante

#define F_LOW0 360   // ← Frecuencias del SLOT BAJO
#define F_LOW1 540

#define UMBRAL_MINIMO 15.0
#define DIFERENCIA_MINIMA 10.0

// ==================== VARIABLES GLOBALES ====================
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

unsigned long samplingPeriod;
bool reproduciendo = false;
int frecuenciaDetectada = 0;

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(FSK_INPUT_PIN, INPUT);
    pinMode(AUDIO_OUTPUT_PIN, OUTPUT);
    
    samplingPeriod = round(1000000.0 / SAMPLING_FREQUENCY);
    
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  RECEPTOR 2 - CANAL AUDIO (360/540 Hz)        ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
}

// ==================== LOOP ====================
void loop() {
    capturarDesdeGPIO();
    aplicarFFT();
    
    int bit = analizarYDemodular();
    
    if (bit != -1) {
        // Detectamos un tono válido
        int freq = (bit == 1) ? F_LOW1 : F_LOW0;
        
        Serial.print("🎵 Tono detectado: ");
        Serial.print(freq);
        Serial.println(" Hz");
        
        reproducirEnParlante(freq);
    } else {
        // Sin señal → silencio
        dacWrite(AUDIO_OUTPUT_PIN, 128);  // Valor medio (silencio)
    }
}

// ==================== FUNCIONES ====================

void capturarDesdeGPIO() {
    unsigned long microseconds;
    for (int i = 0; i < SAMPLES; i++) {
        microseconds = micros();
        
        int level = digitalRead(FSK_INPUT_PIN);
        vReal[i] = (double)level - 0.5;
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
    int index_f0 = round(F_LOW0 * SAMPLES / SAMPLING_FREQUENCY);
    int index_f1 = round(F_LOW1 * SAMPLES / SAMPLING_FREQUENCY);
    
    double mag_f0 = (vReal[index_f0-1] + vReal[index_f0] + vReal[index_f0+1]) / 3.0;
    double mag_f1 = (vReal[index_f1-1] + vReal[index_f1] + vReal[index_f1+1]) / 3.0;
    
    double maxMag = max(mag_f0, mag_f1);
    double diferencia = abs(mag_f0 - mag_f1);
    
    if (maxMag < UMBRAL_MINIMO || diferencia < DIFERENCIA_MINIMA) {
        return -1;  // Sin señal clara
    }
    
    return (mag_f1 > mag_f0) ? 1 : 0;
}

void reproducirEnParlante(int freq) {
    // Generar tono simple en el DAC
    // Frecuencia de actualización del DAC
    static unsigned long lastUpdate = 0;
    static float phase = 0;
    
    unsigned long now = micros();
    float fs = 8000.0;  // Frecuencia de muestreo del DAC
    float dt = 1.0 / fs;
    
    if (now - lastUpdate >= (1000000.0 / fs)) {
        lastUpdate = now;
        
        // Generar onda senoidal
        float sample = 128 + 100 * sin(2 * PI * freq * phase);
        dacWrite(AUDIO_OUTPUT_PIN, constrain((int)sample, 0, 255));
        
        phase += dt;
        if (phase > 1.0) phase -= 1.0;
    }
}
