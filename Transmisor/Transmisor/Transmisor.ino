/*
 * TRANSMISOR FSK DIGITAL - SIN delay()
 * Usa millis()/micros() para timing no bloqueante
 */

#include <Arduino.h>
#include "FS.h"       // define File
#include "SPIFFS.h"   // SPIFFS en ESP32

// ==================== CONFIGURACIÓN ====================
#define TX_PIN 25
#define F1 800.0f      // Hz para bit 0
#define F2 1600.0f     // Hz para bit 1
#define Tb 0.013f      // 13 ms por bit aprox.

// ==================== ESTADO GLOBAL ====================
enum Estado { IDLE, TRANSMITIENDO };
Estado estadoActual = IDLE;

char caracterActual = 0;
int bitIndex = 0;
int cicloActual = 0;
int numCiclosTotal = 0;
bool estadoPin = LOW;

unsigned long tiempoAnterior = 0;
float periodoActual_us = 0;

// ==================== PROTOTIPOS ====================
void iniciarTransmisionBit();
void actualizarTransmision();
void logCaracter(char c);

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Inicializar SPIFFS para log
    if (!SPIFFS.begin(true)) {
        Serial.println("❌ Error montando SPIFFS");
    } else {
        Serial.println("✅ SPIFFS montado");
    }
    
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  TRANSMISOR FSK - SIN delay()         ║");
    Serial.println("║  CE 1110 - Proyecto Comunicación      ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    Serial.println("✅ Sistema listo (modo no bloqueante)");
    Serial.println("📝 Escribe texto para transmitir:\n");
}

// ==================== LOOP ====================
void loop() {
    switch (estadoActual) {
        case IDLE:
            if (Serial.available()) {
                caracterActual = Serial.read();
                
                // Evitar caracteres de control tipo \n \r
                if (caracterActual >= 32) {
                    Serial.print("TX: '");
                    Serial.print(caracterActual);
                    Serial.print("' → ");

                    // Registrar en el log antes de transmitir
                    logCaracter(caracterActual);
                    
                    bitIndex = 7;           // Empezar desde el bit más significativo
                    estadoActual = TRANSMITIENDO;
                    iniciarTransmisionBit();
                }
            }
            break;
            
        case TRANSMITIENDO:
            actualizarTransmision();
            break;
    }
}

// ==================== LOG EN SPIFFS ====================
void logCaracter(char c) {
    int ascii = (int)c;
    
    File logFile = SPIFFS.open("/tx_log.txt", FILE_APPEND);
    if (!logFile) {
        Serial.println("\n⚠️ No se pudo abrir /tx_log.txt para escribir");
        return;
    }

    logFile.print("TX: '");
    logFile.print(c);
    logFile.print("' ASCII:");
    logFile.print(ascii);
    logFile.print(" Bits:");

    // Mismo patrón de bits que se va a transmitir
    for (int i = 7; i >= 0; i--) {
        int bit = (ascii >> i) & 1;
        logFile.print(bit);
    }

    logFile.println();
    logFile.close();
}

// ==================== TRANSMISIÓN NO BLOQUEANTE ====================
void iniciarTransmisionBit() {
    // Bit actual del carácter
    int bit = (((int)caracterActual) >> bitIndex) & 1;
    Serial.print(bit);
    
    // Seleccionar frecuencia según el bit
    float frecuencia = (bit == 0) ? F1 : F2;
    periodoActual_us = 1000000.0f / frecuencia;
    
    // Cuántos ciclos completos caben en el tiempo de bit Tb
    numCiclosTotal = (int)(frecuencia * Tb);
    cicloActual = 0;
    
    // Iniciar onda cuadrada
    estadoPin = HIGH;
    digitalWrite(TX_PIN, estadoPin);
    tiempoAnterior = micros();
}

void actualizarTransmision() {
    unsigned long tiempoActual = micros();
    
    // Cambiar el estado del pin cada medio periodo
    if (tiempoActual - tiempoAnterior >= (periodoActual_us / 2)) {
        tiempoAnterior = tiempoActual;
        
        // Alternar pin
        estadoPin = !estadoPin;
        digitalWrite(TX_PIN, estadoPin);
        
        // Contar ciclos completos cuando volvemos a LOW
        if (estadoPin == LOW) {
            cicloActual++;
            
            // ¿Ya se enviaron todos los ciclos para este bit?
            if (cicloActual >= numCiclosTotal) {
                bitIndex--;
                
                if (bitIndex < 0) {
                    // Carácter completo
                    Serial.println(" ✓");
                    estadoActual = IDLE;
                    digitalWrite(TX_PIN, LOW);
                } else {
                    // Siguiente bit del mismo carácter
                    iniciarTransmisionBit();
                }
            }
        }
    }
}
