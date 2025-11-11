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
#include <LiquidCrystal.h>
// ==================== CONFIGURACIÓN ====================
#define SAMPLES 512
#define SAMPLING_FREQUENCY 4000
#define FSK_INPUT_PIN 34
//#define AUDIO_OUTPUT_PIN 26

#define F1 800
#define F2 1600

#define UMBRAL_MINIMO 15.0    //50 NO, 40 NO, MUY ESTRICTO, 30 SI, 25 MEGA BIEN, 20 FUNCIONA PARA PROYECTO
#define DIFERENCIA_MINIMA 10.0    //20 FUNCIONA, 15 MEJOR CASO

// ==================== VARIABLES GLOBALES ====================
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

unsigned long samplingPeriod;
unsigned long contadorAnalisis = 0;

uint8_t bitsRecibidos[8];
int bitCount = 0;

// RS, E, D4, D5, D6, D7 (Salidas al LCD)
LiquidCrystal lcd(18, 19, 14, 15, 16, 17);



// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);  // Solo inicial
    
    pinMode(FSK_INPUT_PIN, INPUT);
    
    samplingPeriod = round(1000000.0 / SAMPLING_FREQUENCY);

    // Inicializar LCD
    lcd.begin(16, 2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Esperando...");
    
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  RECEPTOR 1 - SIN delay()                     ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
    
    Serial.println("✅ Esperando señal FSK...\n");
}

// ==================== LOOP ====================
void loop() {
    static int sinSenalCount = 0;
    
    // Capturar y procesar
    capturarDesdeGPIO();
    aplicarFFT();
    
    int bit = analizarYDemodular();
    mostrarAnalisis(bit);
    
        if (bit == -1) {
            // No hay tono claro (ni 800 ni 1600)
            sinSenalCount++;
            
            // Si llevamos varias ventanas “vacías”, reseteamos el acumulador de bits
            if (sinSenalCount >= 2) {   // 3*128ms ≈ 384ms de silencio
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
        /*
        Serial.print("\n🔊 CARÁCTER: '");
        Serial.print(c);
        Serial.print("' (ASCII ");
        Serial.print((int)c);
        Serial.println(")\n");
        bitCount = 0;
        */
          // FILTRAR PREÁMBULO
        if (c == 0x55 || c == 'U') {  // 0x55 = 85 decimal = 'U'
            Serial.println("📡 [PREÁMBULO detectado - descartando]");
            bitCount = 0;
            return;  // ← NO imprime el carácter
        }

        Serial.print("\n🔊 CARÁCTER: '");
        Serial.print(c);
        Serial.print("' (ASCII ");
        Serial.print((int)c);
        Serial.println(")\n");
        logLCD(String(c));
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

void logLCD(const String &msg) {
  

  //Mostrar en el LCD
  lcd.clear();
  lcd.setCursor(0,0);

  if(msg.length() <= 16){
    lcd.print(msg);
  } else{
    lcd.print(msg.substring(0,16));
    lcd.setCursor(0,1);
    lcd.print(msg.substring(16,min(32,(int)msg.length())));
  }
}