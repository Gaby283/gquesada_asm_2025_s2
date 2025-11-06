/*
 * RECEPTOR 1 - CORREGIDO
 * Captura bit por bit, sincronizado con Tb
 */

#include "arduinoFFT.h"

// ==================== CONFIGURACIÓN ====================
#define SAMPLES 512
#define SAMPLING_FREQUENCY 4000
#define FSK_INPUT_PIN 34
#define AUDIO_OUTPUT_PIN 26

#define F1 800
#define F2 1600
#define Tb 0.013f  // ← DEBE COINCIDIR CON TRANSMISOR

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

// Para reproducción
bool reproduciendo = false;
int muestraActual = 0;
int totalMuestras = 0;
int frecuenciaAudio = 0;
unsigned long tiempoAnteriorAudio = 0;
const int FS_AUDIO = 8000;
const int PERIODO_AUDIO_US = 1000000 / FS_AUDIO;

// ⭐ NUEVO: Variables para sincronización
unsigned long tiempoUltimoBit = 0;
const unsigned long PERIODO_BIT_MS = (unsigned long)(Tb * 1000);  // 10ms

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(FSK_INPUT_PIN, INPUT);
    
    samplingPeriod = round(1000000.0 / SAMPLING_FREQUENCY);
    
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  RECEPTOR 1 - CORREGIDO (sincronizado)        ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
    
    Serial.print("Configuración:\n");
    Serial.print("  • Tb: ");
    Serial.print(Tb * 1000, 1);
    Serial.println(" ms");
    Serial.print("  • Ventana captura: ");
    Serial.print((float)SAMPLES / SAMPLING_FREQUENCY * 1000, 1);
    Serial.println(" ms");
    
    Serial.println("\n✅ Esperando señal FSK...\n");
    
    tiempoUltimoBit = millis();
}

// ==================== LOOP ====================
void loop() {
    // Si está reproduciendo audio, continuar
    if (reproduciendo) {
        actualizarAudio();
        return;
    }
    
    // ⭐ CAMBIO: Solo capturar cada Tb milisegundos
    unsigned long tiempoActual = millis();
    if (tiempoActual - tiempoUltimoBit >= PERIODO_BIT_MS) {
        tiempoUltimoBit = tiempoActual;
        
        // Capturar ventana pequeña (solo un bit)
        capturarVentanaBit();
        aplicarFFT();
        
        int bit = analizarYDemodular();
        mostrarAnalisis(bit);
        
        if (bit != -1) {
            acumularBit(bit);
        } else {
            // Sin señal válida, resetear si pasa mucho tiempo
            if (bitCount > 0) {
                Serial.println("⚠️  Señal perdida, reseteando buffer");
                bitCount = 0;
            }
        }
        
        contadorAnalisis++;
    }
}

// ==================== FUNCIONES ====================

// ⭐ NUEVA FUNCIÓN: Captura solo una ventana del tamaño de un bit
void capturarVentanaBit() {
    // Calcular cuántas muestras necesitamos para Tb segundos
    int muestrasParaTb = (int)(Tb * SAMPLING_FREQUENCY);  // 40 muestras @ Tb=0.01, Fs=4000
    
    // Capturar solo esas muestras
    unsigned long microseconds;
    for (int i = 0; i < muestrasParaTb && i < SAMPLES; i++) {
        microseconds = micros();
        
        int level = digitalRead(FSK_INPUT_PIN);
        vReal[i] = (double)level - 0.5;
        vImag[i] = 0;
        
        while (micros() - microseconds < samplingPeriod) {
            // espera activa
        }
    }
    
    // Rellenar el resto con ceros (zero-padding para FFT)
    for (int i = muestrasParaTb; i < SAMPLES; i++) {
        vReal[i] = 0;
        vImag[i] = 0;
    }
}

void capturarDesdeGPIO() {
    // MANTENER PARA COMPATIBILIDAD
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
    int index_f1 = round(F1 * SAMPLES / SAMPLING_FREQUENCY);
    int index_f2 = round(F2 * SAMPLES / SAMPLING_FREQUENCY);
    
    // Verificar que índices estén en rango
    if (index_f1 < 1 || index_f2 < 1 || index_f1 >= SAMPLES-1 || index_f2 >= SAMPLES-1) {
        return -1;
    }
    
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
    
    if (index_f1 < 1 || index_f2 < 1 || index_f1 >= SAMPLES-1 || index_f2 >= SAMPLES-1) {
        Serial.println("❌ Error: índices fuera de rango");
        return;
    }
    
    double mag_f1 = (vReal[index_f1-1] + vReal[index_f1] + vReal[index_f1+1]) / 3.0;
    double mag_f2 = (vReal[index_f2-1] + vReal[index_f2] + vReal[index_f2+1]) / 3.0;
    
    Serial.print("Bit #");
    Serial.print(contadorAnalisis);
    Serial.print(" | 800Hz: ");
    Serial.print(mag_f1, 0);
    Serial.print(" | 1600Hz: ");
    Serial.print(mag_f2, 0);
    Serial.print(" → ");
    
    if (bit == -1) {
        Serial.println("❌");
    } else {
        Serial.print(bit);
        Serial.print(" [");
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
        
        Serial.print("\n🔊 CARÁCTER COMPLETO: '");
        Serial.print(c);
        Serial.print("' (ASCII ");
        Serial.print((int)c);
        Serial.print(") Binario: ");
        for (int i = 0; i < 8; i++) {
            Serial.print(bitsRecibidos[i]);
        }
        Serial.println("\n");
        
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
    totalMuestras = (300 * FS_AUDIO) / 1000;
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
            dacWrite(AUDIO_OUTPUT_PIN, 128);
            reproduciendo = false;
        }
    }
}