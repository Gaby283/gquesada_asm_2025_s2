/*
 * RECEPTOR 2 - Con audio continuo (solución mejorada)
 */

#include "arduinoFFT.h"

#define SAMPLES 512
#define SAMPLING_FREQUENCY 4000
#define FSK_INPUT_PIN 34
#define AUDIO_OUTPUT_PIN 26

#define F_LOW0 360
#define F_LOW1 540

#define UMBRAL_MINIMO 15.0
#define DIFERENCIA_MINIMA 10.0

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

unsigned long samplingPeriod;

// Variables para audio continuo
float freqFiltrada = 450.0;
const float ALPHA = 0.2;
float audioPhase = 0;
const float AUDIO_FS = 8000.0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(FSK_INPUT_PIN, INPUT);
    pinMode(AUDIO_OUTPUT_PIN, OUTPUT);
    
    samplingPeriod = round(1000000.0 / SAMPLING_FREQUENCY);
    
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  RECEPTOR 2 - FSK PILOTO (Audio Continuo)     ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
}

void loop() {
    // Capturar MIENTRAS se reproduce audio
    capturarConAudio();
    
    // Procesar FFT
    aplicarFFT();
    
    // Actualizar frecuencia filtrada
    int bit = analizarYDemodular();
    
    if (bit != -1) {
        float freqDetectada = (bit == 1) ? F_LOW1 : F_LOW0;
        freqFiltrada = ALPHA * freqDetectada + (1.0 - ALPHA) * freqFiltrada;
        
        Serial.print("🎵 FSK: ");
        Serial.print((int)freqDetectada);
        Serial.print(" Hz → Filtrado: ");
        Serial.print((int)freqFiltrada);
        Serial.println(" Hz");
    } else {
        // Decay suave cuando no hay señal
        freqFiltrada = 0.95 * freqFiltrada;
        
        if (freqFiltrada < 50) {
            freqFiltrada = 0;  // Silencio total
        }
    }
}

// NUEVA FUNCIÓN: Captura Y reproduce simultáneamente
void capturarConAudio() {
    unsigned long lastAudioSample = micros();
    
    for (int i = 0; i < SAMPLES; i++) {
        unsigned long microseconds = micros();
        
        // Leer muestra para FFT
        int level = digitalRead(FSK_INPUT_PIN);
        vReal[i] = (double)level - 0.5;
        vImag[i] = 0;
        
        // MIENTRAS esperamos el siguiente sample de FFT,
        // generar VARIAS muestras de audio
        while (micros() - microseconds < samplingPeriod) {
            unsigned long now = micros();
            
            // Generar audio cada 125 μs (8 kHz)
            if (now - lastAudioSample >= 125) {
                lastAudioSample = now;
                
                if (freqFiltrada > 50) {  // Solo si hay señal
                    float sample = 128 + 100 * sin(2 * PI * freqFiltrada * audioPhase);
                    dacWrite(AUDIO_OUTPUT_PIN, constrain((int)sample, 0, 255));
                    
                    audioPhase += 1.0 / AUDIO_FS;
                    if (audioPhase > 1.0) audioPhase -= 1.0;
                } else {
                    dacWrite(AUDIO_OUTPUT_PIN, 128);  // Silencio
                }
            }
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
        return -1;
    }
    
    return (mag_f1 > mag_f0) ? 1 : 0;
}
