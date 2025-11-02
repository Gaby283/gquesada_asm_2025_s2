/*
 * Demodulador FSK con FFT para ESP32 - ARRAYS PRE-CARGADOS
 * Proyecto: Sistema de Comunicación Local - CE 1110
 * 
 * Descripción:
 * Este código implementa un demodulador FSK que:
 * 1. Usa arrays pre-cargados (sin necesidad de circuito externo)
 * 2. Aplica FFT para análisis espectral
 * 3. Detecta las frecuencias dominantes (f1=800Hz, f2=1600Hz)
 * 4. Muestra resultados por Serial Monitor
 * 
 * INSTRUCCIONES:
 * 1. Ejecuta tu modulacion.py para generar arrays_esp32.txt
 * 2. Copia los 3 arrays generados y pégalos donde dice "PEGAR ARRAYS AQUÍ"
 * 3. Compila y sube al ESP32
 */

#include "arduinoFFT.h"

// ==================== CONFIGURACIÓN ====================
#define SAMPLES 512              // Muestras para FFT (debe coincidir con Python)
#define SAMPLING_FREQUENCY 8000  // 8 kHz (debe coincidir con Python)

// Frecuencias FSK
#define F1 800   // Frecuencia para bit 0
#define F2 1600  // Frecuencia para bit 1

// ==================== PEGAR ARRAYS AQUÍ ====================
// Copia los arrays generados por modulacion.py aquí:

// Ejemplo (REEMPLAZAR con tus arrays):
// ========== SENAL BIT 0 (800 Hz) ==========
// ========== SENAL BIT 0 (800 Hz PURO) ==========
const int senal_bit0[512] = {
    2048, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2048,
     782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313,
    4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2048,  782,    1,
       1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313, 4094, 4094,
    3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782,
    2047, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2047,
     782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313,
    4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2047,  782,    1,
       1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094,
    3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782,
    2047, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,
     782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2048, 3313,
    4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,
       1,  782, 2048, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094,
    3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782,
    2047, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2048,
     782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2048, 3313,
    4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,
       1,  782, 2048, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094,
    3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782,
    2047, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2047,
     782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313,
    4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2047,  782,    1,
       1,  782, 2047, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094,
    3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782,
    2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,
     782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2047, 3313,
    4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,
       1,  782, 2047, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2048, 3313, 4095, 4094,
    3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782,
    2047, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,
     782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2047, 3313
};


// ========== SENAL BIT 1 (1600 Hz PURO) ==========
const int senal_bit1[512] = {
    2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2047,
    4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094,
    3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2047, 4094, 3313,
     782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,
       1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1,
    2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048,
    4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094,
    3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,
     782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,
       1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1,
    2047, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048,
    4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094,
    3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,
     782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,
       1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1,
    2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047,
    4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094,
    3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,
     782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,
       1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1,
    2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048,
    4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094,
    3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,
     782,    1, 2047, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,
       1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1,
    2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048,
    4095, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2047, 4094,
    3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,
     782,    1, 2047, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,
       1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1,
    2047, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048,
    4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2047, 4094
};


// ========== SENAL MIXTA (800Hz + 1600Hz) ==========
const int senal_mixta[512] = {
    2048, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2048,
     782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313,
    4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2048,  782,    1,
       1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313, 4094, 4094,
    3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782,
    2047, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2047,
     782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313,
    4094, 4094, 3313, 2047,  782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2047,  782,    1,
       1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094,
    3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782,
    2047, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,
     782,    1,    1,  782, 2047, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2048, 3313,
    4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,
       1,  782, 2048, 3313, 4094, 4094, 3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094,
    3313, 2048,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782,
    2047, 3313, 4094, 4094, 3313, 2047,  782,    1,    1,  782, 2048, 3313, 4094, 4094, 3313, 2048,
    2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2047,
    4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094,
    3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2047, 4094, 3313,
     782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,
       1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1,
    2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048,
    4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094,
    3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,
     782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,
       1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1,
    2047, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048,
    4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094,
    3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,
     782,    1, 2048, 4094, 3313,  782,    1, 2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,
       1, 2047, 4094, 3313,  782,    1, 2048, 4095, 3313,  782,    1, 2048, 4094, 3313,  782,    1,
    2047, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2048, 4094, 3313,  782,    1, 2047
};


// ==================== VARIABLES GLOBALES ====================
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

int modoActual = 0;  // 0: bit0, 1: bit1, 2: mixta
unsigned long contadorAnalisis = 0;

// ==================== FUNCIONES ====================

void cargarArray(const int* array_fuente) {
    // Calcular promedio (componente DC)
    long suma = 0;
    for (int i = 0; i < SAMPLES; i++) {
        suma += array_fuente[i];
    }
    double promedio = suma / (double)SAMPLES;
    
    // Cargar y centrar, eliminando DC
    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = array_fuente[i] - promedio;  // ← IMPORTANTE
        vImag[i] = 0;
    }
}

void aplicarFFT() {
    // Aplicar ventana de Hamming
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    
    // Calcular FFT
    FFT.compute(FFTDirection::Forward);
    
    // Convertir a magnitud
    FFT.complexToMagnitude();
}

void analizarEspectro(const char* nombre_senal) {
    // Calcular índices de frecuencia
    int index_f1 = round(F1 * SAMPLES / SAMPLING_FREQUENCY);
    int index_f2 = round(F2 * SAMPLES / SAMPLING_FREQUENCY);
    
    // Obtener magnitudes (promediar bins adyacentes para mayor precisión)
    double mag_f1 = (vReal[index_f1-1] + vReal[index_f1] + vReal[index_f1+1]) / 3.0;
    double mag_f2 = (vReal[index_f2-1] + vReal[index_f2] + vReal[index_f2+1]) / 3.0;
    
    // Encontrar frecuencia dominante
    double peakFrequency = FFT.majorPeak();
    
    // Demodular bit
    int bitDemodulado = (mag_f2 > mag_f1) ? 1 : 0;
    
    // ========== MOSTRAR RESULTADOS ==========
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.print("║  Análisis #");
    Serial.print(contadorAnalisis);
    Serial.print(" - ");
    Serial.print(nombre_senal);
    for(int i = strlen(nombre_senal); i < 30; i++) Serial.print(" ");
    Serial.println("║");
    Serial.println("╠════════════════════════════════════════════════╣");
    
    // Frecuencia dominante
    Serial.print("║  Frecuencia dominante: ");
    Serial.print(peakFrequency, 1);
    Serial.print(" Hz");
    for(int i = String(peakFrequency, 1).length(); i < 24; i++) Serial.print(" ");
    Serial.println("║");
    
    Serial.println("║                                                ║");
    
    // Magnitudes en f1 y f2
    Serial.print("║  Magnitud @ ");
    Serial.print(F1);
    Serial.print(" Hz:  ");
    Serial.print(mag_f1, 2);
    if (mag_f1 > mag_f2) Serial.print(" ← MAYOR");
    for(int i = String(mag_f1, 2).length() + (mag_f1 > mag_f2 ? 9 : 0); i < 21; i++) Serial.print(" ");
    Serial.println("║");
    
    Serial.print("║  Magnitud @ ");
    Serial.print(F2);
    Serial.print(" Hz: ");
    Serial.print(mag_f2, 2);
    if (mag_f2 > mag_f1) Serial.print(" ← MAYOR");
    for(int i = String(mag_f2, 2).length() + (mag_f2 > mag_f1 ? 9 : 0); i < 21; i++) Serial.print(" ");
    Serial.println("║");
    
    Serial.println("║                                                ║");
    
    // Bit demodulado
    Serial.print("║  ➤ BIT DEMODULADO: ");
    if (bitDemodulado == 0) {
        Serial.print("0 (800 Hz)");
    } else {
        Serial.print("1 (1600 Hz)");
    }
    for(int i = (bitDemodulado == 0 ? 10 : 11); i < 24; i++) Serial.print(" ");
    Serial.println("║");
    
    Serial.println("╠════════════════════════════════════════════════╣");
    Serial.println("║  TOP 5 FRECUENCIAS DEL ESPECTRO               ║");
    Serial.println("╠════════════════════════════════════════════════╣");
    
    mostrarTopFrecuencias(5);
    
    Serial.println("╚════════════════════════════════════════════════╝\n");
}

void mostrarTopFrecuencias(int n) {
    // Arrays para almacenar top n frecuencias
    double freqs[n];
    double mags[n];
    
    // Inicializar
    for (int i = 0; i < n; i++) {
        freqs[i] = 0;
        mags[i] = 0;
    }
    
    // Buscar los n picos más grandes (solo frecuencias positivas)
    for (int i = 1; i < SAMPLES/2; i++) {
        double freq = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
        double mag = vReal[i];
        
        // Insertar en orden
        for (int j = 0; j < n; j++) {
            if (mag > mags[j]) {
                // Desplazar valores
                for (int k = n-1; k > j; k--) {
                    mags[k] = mags[k-1];
                    freqs[k] = freqs[k-1];
                }
                // Insertar nuevo
                mags[j] = mag;
                freqs[j] = freq;
                break;
            }
        }
    }
    
    // Mostrar resultados con formato
    for (int i = 0; i < n; i++) {
        Serial.print("║  ");
        Serial.print(i+1);
        Serial.print(". ");
        
        char buffer[40];
        sprintf(buffer, "%-8.1f Hz → Mag: %-10.2f", freqs[i], mags[i]);
        Serial.print(buffer);
        
        for(int j = strlen(buffer); j < 36; j++) Serial.print(" ");
        Serial.println("║");
    }
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n");
    Serial.println("╔══════════════════════════════════════════════════════╗");
    Serial.println("║  Demodulador FSK con FFT - ESP32                    ║");
    Serial.println("║  Arrays Pre-cargados (sin circuito externo)         ║");
    Serial.println("║  CE 1110 - Análisis de Señales Mixtas               ║");
    Serial.println("╚══════════════════════════════════════════════════════╝");
    
    Serial.println("\n📋 CONFIGURACIÓN:");
    Serial.println("─────────────────────────────────────────────────────");
    Serial.print("  • Frecuencia de muestreo: ");
    Serial.print(SAMPLING_FREQUENCY);
    Serial.println(" Hz");
    Serial.print("  • Muestras FFT: ");
    Serial.println(SAMPLES);
    Serial.print("  • Resolución frecuencial: ");
    Serial.print((double)SAMPLING_FREQUENCY / SAMPLES, 2);
    Serial.println(" Hz");
    Serial.print("  • F1 (bit 0): ");
    Serial.print(F1);
    Serial.println(" Hz");
    Serial.print("  • F2 (bit 1): ");
    Serial.print(F2);
    Serial.println(" Hz");
    Serial.println("─────────────────────────────────────────────────────");
    
    Serial.println("\n✅ Sistema listo.");
    Serial.println("📊 Analizando señales cada 3 segundos...\n");
    
    delay(2000);
}

// ==================== LOOP ====================
void loop() {
    const char* nombres[] = {"Señal BIT 0 (800 Hz)", "Señal BIT 1 (1600 Hz)", "Señal MIXTA"};
    const int* arrays[] = {senal_bit0, senal_bit1, senal_mixta};
    
    // Cargar array según modo actual
    cargarArray(arrays[modoActual]);
    
    // Aplicar FFT
    aplicarFFT();
    
    // Analizar y mostrar resultados
    analizarEspectro(nombres[modoActual]);
    
    contadorAnalisis++;
    
    // Cambiar al siguiente modo
    modoActual = (modoActual + 1) % 3;
    
    // Pausa entre análisis
    delay(3000);
}