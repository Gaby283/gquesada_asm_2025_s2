/*
 * TRANSMISOR FSK DIGITAL - SIN delay()
 * Usa millis() para timing no bloqueante
 */

#include <Arduino.h>

#define TX_PIN 25
#define F1 800.0f
#define F2 1600.0f
#define Tb 0.140f          // 140 ms por bit
#define T_GUARDA 0.30f     // 300 ms de silencio entre caracteres

// ==================== VARIABLES GLOBALES ====================
enum Estado { IDLE, TRANSMITIENDO, GUARDA };

Estado estadoActual = IDLE;

char caracterActual = 0;
int bitIndex = 0;
int cicloActual = 0;
int numCiclosTotal = 0;
bool estadoPin = LOW;

unsigned long tiempoAnterior = 0;
float periodoActual_us = 0;

unsigned long inicioGuarda_ms = 0;   // NUEVO: para el estado GUARDA

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);  // Solo este delay inicial está OK
    
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  TRANSMISOR FSK - SIN delay()         ║");
    Serial.println("║  CE 1110 - Proyecto Comunicación      ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    Serial.println("✅ Sistema listo (modo no bloqueante)");
    Serial.println("📝 Escribe texto para transmitir:\n");
}

void loop() {
    switch (estadoActual) {
        case IDLE:
            if (Serial.available()) {
                caracterActual = Serial.read();
                
                if (caracterActual >= 32) {
                    Serial.print("TX: '");
                    Serial.print(caracterActual);
                    Serial.print("' → ");
                    
                    bitIndex = 7;
                    estadoActual = TRANSMITIENDO;
                    iniciarTransmisionBit();
                }
            }
            break;
            
        case TRANSMITIENDO:
            actualizarTransmision();
            break;

        case GUARDA: {
            // Mantener la línea en LOW un tiempo fijo
            unsigned long ahora = millis();
            if (ahora - inicioGuarda_ms >= (unsigned long)(T_GUARDA * 1000)) {
                estadoActual = IDLE;
            }
            break;
        }
    }
}

// ==================== FUNCIONES ====================

void iniciarTransmisionBit() {
    // Obtener bit actual
    int bit = (((int)caracterActual) >> bitIndex) & 1;
    Serial.print(bit);
    
    // Configurar frecuencia
    float frecuencia = (bit == 0) ? F1 : F2;
    periodoActual_us = 1000000.0 / frecuencia;
    
    // Calcular ciclos necesarios para Tb
    numCiclosTotal = (int)(frecuencia * Tb);
    cicloActual = 0;
    
    // Iniciar onda cuadrada
    estadoPin = HIGH;
    digitalWrite(TX_PIN, estadoPin);
    tiempoAnterior = micros();
}

void actualizarTransmision() {
    unsigned long tiempoActual = micros();
    
    // Verificar si pasó medio periodo
    if (tiempoActual - tiempoAnterior >= (periodoActual_us / 2)) {
        tiempoAnterior = tiempoActual;
        
        // Alternar pin
        estadoPin = !estadoPin;
        digitalWrite(TX_PIN, estadoPin);
        
        // Si completó un ciclo completo (HIGH→LOW)
        if (estadoPin == LOW) {
            cicloActual++;
            
            // Verificar si completó todos los ciclos del bit
            if (cicloActual >= numCiclosTotal) {
                // Bit completado, pasar al siguiente
                bitIndex--;
                
                if (bitIndex < 0) {
                    // Carácter completo
                    Serial.println(" ✓");
                    digitalWrite(TX_PIN, LOW);

                    // Entrar a estado de guarda (silencio controlado)
                    inicioGuarda_ms = millis();
                    estadoActual = GUARDA;
                } else {
                    // Siguiente bit
                    iniciarTransmisionBit();
                }

            }
        }
    }
}