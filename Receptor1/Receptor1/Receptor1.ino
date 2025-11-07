/*
 * RECEPTOR 1 - Demodulador FSK con PARLANTE
 * Proyecto: Sistema de Comunicación Local - CE 1110
 * 
 * Hardware:
 * - Pin GPIO34: Entrada digital FSK (onda cuadrada 0/1)
 * - Pin GPIO26 (DAC2): Salida a parlante/amplificador

*/
/*
 * RECEPTOR 1 - SIN delay()
 * Usa millis() para timing no bloqueante
 */

#include "arduinoFFT.h"

// ==================== CONFIGURACIÓN ====================
#define SAMPLES 512
#define SAMPLING_FREQUENCY 4000
#define FSK_INPUT_PIN 34
#define AUDIO_OUTPUT_PIN 26

#define F1 800
#define F2 1600

#define UMBRAL_MINIMO 50.0
#define DIFERENCIA_MINIMA 20.0

// ==================== VARIABLES GLOBALES ====================
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

unsigned long samplingPeriod;
unsigned long contadorAnalisis = 0;

uint8_t bitsRecibidos[8];
int bitCount = 0;

// Variables para reproducción de audio sin delay
bool reproduciendo = false;
int muestraActual = 0;
int totalMuestras = 0;
int frecuenciaAudio = 0;
unsigned long tiempoAnteriorAudio = 0;
const int FS_AUDIO = 8000;
const int PERIODO_AUDIO_US = 1000000 / FS_AUDIO;  // 125 μs

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);  // Solo inicial
    
    pinMode(FSK_INPUT_PIN, INPUT);
    
    samplingPeriod = round(1000000.0 / SAMPLING_FREQUENCY);
    
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  RECEPTOR 1 - SIN delay()                     ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
    
    Serial.println("✅ Esperando señal FSK...\n");
}

// ==================== LOOP ====================
void loop() {
    static int sinSenalCount = 0;
    // Si está reproduciendo audio, continuar
    if (reproduciendo) {
        actualizarAudio();
        //return;  // **No capturar mientras reproduce      ***PARA ARREGLAR, PRUEBAS SIN EL RETURN
    }
    
    // Capturar y procesar
    capturarDesdeGPIO();
    aplicarFFT();
    
    int bit = analizarYDemodular();
    mostrarAnalisis(bit);
    
        if (bit == -1) {
            // No hay tono claro (ni 800 ni 1600)
            sinSenalCount++;
            
            // Si llevamos varias ventanas “vacías”, reseteamos el acumulador de bits
            if (sinSenalCount >= 3) {   // 3*128ms ≈ 384ms de silencio
                bitCount = 0;
            }
        } else {
            // Hay bit válido → reseteo contador de silencio y acumulo
            sinSenalCount = 0;
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
        
        Serial.print("\n🔊 CARÁCTER: '");
        Serial.print(c);
        Serial.print("' (ASCII ");
        Serial.print((int)c);
        Serial.println(")\n");
        
        iniciarReproduccion(c);
        
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

void iniciarReproduccion(char c) {
    frecuenciaAudio = 200 + ((int)c * 10);
    totalMuestras = (300 * FS_AUDIO) / 1000;  // 300ms
    muestraActual = 0;
    reproduciendo = true;
    tiempoAnteriorAudio = micros();
}

void actualizarAudio() {
    unsigned long tiempoActual = micros();
    
    if (tiempoActual - tiempoAnteriorAudio >= PERIODO_AUDIO_US) {
        tiempoAnteriorAudio = tiempoActual;
        
        if (muestraActual < totalMuestras) {
            float t = muestraActual / (float)FS_AUDIO;
            int valor = 128 + 100 * sin(2 * PI * frecuenciaAudio * t);
            dacWrite(AUDIO_OUTPUT_PIN, constrain(valor, 0, 255));
            muestraActual++;
        } else {
            // Terminar reproducción
            dacWrite(AUDIO_OUTPUT_PIN, 128);
            reproduciendo = false;
        }
    }
}
