/*
 * RECEPTOR 2 - Demodulador FSK con PARLANTE
 * Proyecto: Sistema de Comunicación Local - CE 1110
*/
#include "arduinoFFT.h"

// ==================== CONFIGURACIÓN ====================
#define SAMPLES 512
#define SAMPLING_FREQUENCY 4000
#define FSK_INPUT_PIN 34
#define AUDIO_OUTPUT_PIN 26

// Frecuencias que vienen del transmisor para el canal audio
#define F1 400    // bit 0
#define F2 1200   // bit 1

// Frecuencias del seno que quieres escuchar en el parlante
#define FREQ_AUDIO_GRAVE 300   // Hz cuando se detecta F1 (400 Hz)
#define FREQ_AUDIO_AGUDA 800   // Hz cuando se detecta F2 (1200 Hz)

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
int frecuenciaAudio = 0;
unsigned long tiempoAnteriorAudio = 0;
const int FS_AUDIO = 8000;
const int PERIODO_AUDIO_US = 1000000 / FS_AUDIO;  // 125 μs

// Para decidir cuándo apagar el audio por silencio
int sinSenalCount = 0;

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(FSK_INPUT_PIN, INPUT);
    
    samplingPeriod = round(1000000.0 / SAMPLING_FREQUENCY);
    
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  RECEPTOR 2 - AUDIO FSK (sin delay)           ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
    
    Serial.println("✅ Esperando señal FSK para audio...\n");
}

// ==================== LOOP ====================
void loop() {
    // Seguir generando audio si está activo
    if (reproduciendo) {
        actualizarAudio();
    }
    
    // Capturar y procesar ventana
    capturarDesdeGPIO();
    aplicarFFT();
    
    int bit = analizarYDemodular();
    mostrarAnalisis(bit);
    
    if (bit == -1) {
        // No hay tono claro (ni F1 ni F2)
        sinSenalCount++;
        
        // Si llevamos varias ventanas sin tono, apagamos el audio
        if (sinSenalCount >= 3) {
            reproduciendo = false;
            frecuenciaAudio = 0;
            dacWrite(AUDIO_OUTPUT_PIN, 128);  // nivel medio (silencio)
        }
    } else {
        // Hay bit válido → elegir frecuencia de audio
        sinSenalCount = 0;
        
        if (bit == 0) {
            frecuenciaAudio = FREQ_AUDIO_GRAVE;
        } else {
            frecuenciaAudio = FREQ_AUDIO_AGUDA;
        }
        reproduciendo = true;   // aseguramos que el audio esté activo
    }
    
    contadorAnalisis++;
}

// ==================== FUNCIONES ====================

void capturarDesdeGPIO() {
    unsigned long microseconds;
    for (int i = 0; i < SAMPLES; i++) {
        microseconds = micros();

        int level = digitalRead(FSK_INPUT_PIN);  // 0 ó 1 desde el cable

        // Centramos en 0: 0 -> -0.5, 1 -> +0.5
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
    int index_f1 = round(F1 * SAMPLES / SAMPLING_FREQUENCY);
    int index_f2 = round(F2 * SAMPLES / SAMPLING_FREQUENCY);
    
    double mag_f1 = (vReal[index_f1-1] + vReal[index_f1] + vReal[index_f1+1]) / 3.0;
    double mag_f2 = (vReal[index_f2-1] + vReal[index_f2] + vReal[index_f2+1]) / 3.0;
    
    double maxMag = max(mag_f1, mag_f2);
    double diferencia = abs(mag_f1 - mag_f2);
    
    if (maxMag < UMBRAL_MINIMO || diferencia < DIFERENCIA_MINIMA) {
        return -1;  // sin decisión clara
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
    Serial.print(" | Mag@");
    Serial.print(F1);
    Serial.print(": ");
    Serial.print(mag_f1, 0);
    Serial.print(" | Mag@");
    Serial.print(F2);
    Serial.print(": ");
    Serial.print(mag_f2, 0);
    Serial.print(" | Bit: ");
    
    if (bit == -1) {
        Serial.println("❌");
    } else {
        Serial.println(bit);
    }
}

void actualizarAudio() {
    if (frecuenciaAudio <= 0) return;  // nada que reproducir
    
    unsigned long tiempoActual = micros();
    
    if (tiempoActual - tiempoAnteriorAudio >= PERIODO_AUDIO_US) {
        tiempoAnteriorAudio = tiempoActual;
        
        // t en segundos desde "0" (no necesitamos muestraActual, basta con usar micros())
        static unsigned long faseContador = 0;
        faseContador++;
        float t = (float)faseContador / FS_AUDIO;
        
        int valor = 128 + 100 * sin(2 * PI * frecuenciaAudio * t);
        dacWrite(AUDIO_OUTPUT_PIN, constrain(valor, 0, 255));
    }
}
