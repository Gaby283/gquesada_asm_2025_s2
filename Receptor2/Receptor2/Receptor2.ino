/*
 * RECEPTOR 2 - Demodulador FSK con LCD
 * ESP32 #3 - Muestra texto en LCD
 * 
 * Hardware:
 * - GPIO34: Entrada señal FSK DIGITAL (onda cuadrada)
 * - LCD 16x2 I2C: SDA=GPIO21, SCL=GPIO22
 */

#include "arduinoFFT.h"
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==================== CONFIGURACIÓN ====================
#define SAMPLES 512
#define SAMPLING_FREQUENCY 4000      // 4 kHz (ajustado)
#define FSK_INPUT_PIN 34
#define F1 800
#define F2 1600

#define UMBRAL_MINIMO 50.0
#define DIFERENCIA_MINIMA 20.0

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

unsigned long samplingPeriod;
uint8_t bitsRecibidos[8];
int bitCount = 0;
String mensajeCompleto = "";

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    pinMode(FSK_INPUT_PIN, INPUT);
    
    samplingPeriod = round(1000000.0 / SAMPLING_FREQUENCY);
    
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("RECEPTOR 2 - LCD");
    lcd.setCursor(0, 1);
    lcd.print("Esperando...");
    
    Serial.println("Receptor 2 - LCD listo");
    delay(2000);
    lcd.clear();
}

// ==================== LOOP ====================
void loop() {
    capturarDesdeGPIO();
    
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
    
    int bit = demodular();
    
    if (bit != -1) {
        acumularBit(bit);
    }
}

// ==================== FUNCIONES ====================

void capturarDesdeGPIO() {
    unsigned long microseconds;
    for (int i = 0; i < SAMPLES; i++) {
        microseconds = micros();
        
        int level = digitalRead(FSK_INPUT_PIN);
        vReal[i] = (level == 1) ? 1.0 : -1.0;  // ⭐ CORREGIDO
        vImag[i] = 0.0;
        
        while (micros() - microseconds < samplingPeriod) {
            // espera activa
        }
    }
}

int demodular() {
    int idx1 = round(F1 * SAMPLES / SAMPLING_FREQUENCY);
    int idx2 = round(F2 * SAMPLES / SAMPLING_FREQUENCY);
    
    double mag1 = (vReal[idx1 - 1] + vReal[idx1] + vReal[idx1 + 1]) / 3.0;
    double mag2 = (vReal[idx2 - 1] + vReal[idx2] + vReal[idx2 + 1]) / 3.0;
    
    double maxMag = max(mag1, mag2);
    double diferencia = fabs(mag1 - mag2);
    
    if (maxMag < UMBRAL_MINIMO || diferencia < DIFERENCIA_MINIMA) {
        return -1;
    }
    
    return (mag2 > mag1) ? 1 : 0;
}

void acumularBit(int bit) {
    bitsRecibidos[bitCount++] = bit;
    
    if (bitCount >= 8) {
        char c = bitsToChar();
        
        Serial.print("Carácter: ");
        Serial.println(c);
        
        mostrarEnLCD(c);
        
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

void mostrarEnLCD(char c) {
    mensajeCompleto += c;
    
    lcd.clear();
    int len = mensajeCompleto.length();
    
    if (len <= 16) {
        lcd.setCursor(0, 0);
        lcd.print(mensajeCompleto);
    } else if (len <= 32) {
        lcd.setCursor(0, 0);
        lcd.print(mensajeCompleto.substring(0, 16));
        lcd.setCursor(0, 1);
        lcd.print(mensajeCompleto.substring(16));
    } else {
        // Scroll: últimos 32 caracteres
        lcd.setCursor(0, 0);
        lcd.print(mensajeCompleto.substring(len - 32, len - 16));
        lcd.setCursor(0, 1);
        lcd.print(mensajeCompleto.substring(len - 16));
    }
}