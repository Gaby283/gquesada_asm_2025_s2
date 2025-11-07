/*
 * TRANSMISOR FSK DIGITAL - SIN delay()
 * Un solo GPIO envía:
 *  - Texto (FSK 800/1600) para Receptor1
 *  - Comandos de audio (FSK 400/1200) para Receptor2
 */

#include <Arduino.h>

#define TX_PIN 25

// Portadoras para TEXTO (Receptor1)
#define F_TX0 800.0f     // bit 0
#define F_TX1 1600.0f    // bit 1

// Portadoras para AUDIO (Receptor2)
#define F_AUDIO_LOW  400.0f   // comando '0'
#define F_AUDIO_HIGH 1200.0f  // comando '1'

// Tiempos
#define Tb       0.140f   // 140 ms por bit (texto)
#define T_GUARDA 0.30f    // 300 ms de silencio entre caracteres
#define T_AUDIO  0.50f    // 500 ms de ráfaga para cada comando de audio

// ==================== VARIABLES GLOBALES ====================
enum Estado { IDLE, TRANSMITIENDO_TEXTO, TRANSMITIENDO_AUDIO, GUARDA };
Estado estadoActual = IDLE;

char caracterActual = 0;
int bitIndex = 0;
int cicloActual = 0;
int numCiclosTotal = 0;
bool estadoPin = LOW;

unsigned long tiempoAnterior = 0;
float periodoActual_us = 0;

unsigned long inicioGuarda_ms = 0;
unsigned long inicioAudio_ms = 0;

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  TRANSMISOR FSK - SIN delay()         ║");
    Serial.println("║  CE 1110 - Proyecto Comunicación      ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    Serial.println("✅ Sistema listo");
    Serial.println("   • Letras normales → TEXTO (800/1600)");
    Serial.println("   • '0' → AUDIO grave (400 Hz)");
    Serial.println("   • '1' → AUDIO agudo (1200 Hz)\n");
}

// ==================== LOOP ====================
void loop() {
    switch (estadoActual) {
        case IDLE:
            if (Serial.available()) {
                caracterActual = Serial.read();

                // Comandos de AUDIO: '0' y '1'
                if (caracterActual == '0' || caracterActual == '1') {
                    iniciarTransmisionAudio(caracterActual);
                
                // TEXTO ASCII normal (espacio en adelante)
                } else if (caracterActual >= 32) {
                    Serial.print("TX TEXTO: '");
                    Serial.print(caracterActual);
                    Serial.print("' → ");

                    bitIndex = 7;  // MSB primero
                    estadoActual = TRANSMITIENDO_TEXTO;
                    iniciarTransmisionBitTexto();
                }
            }
            break;
            
        case TRANSMITIENDO_TEXTO:
            actualizarTransmisionTexto();
            break;

        case TRANSMITIENDO_AUDIO:
            actualizarTransmisionAudio();
            break;

        case GUARDA: {
            unsigned long ahora = millis();
            if (ahora - inicioGuarda_ms >= (unsigned long)(T_GUARDA * 1000)) {
                estadoActual = IDLE;
            }
            break;
        }
    }
}

// ==================== FUNCIONES TEXTO ====================

void iniciarTransmisionBitTexto() {
    int bit = (((int)caracterActual) >> bitIndex) & 1;
    Serial.print(bit);
    
    float frecuencia = (bit == 0) ? F_TX0 : F_TX1;
    periodoActual_us = 1000000.0 / frecuencia;
    
    numCiclosTotal = (int)(frecuencia * Tb);
    cicloActual = 0;
    
    estadoPin = HIGH;
    digitalWrite(TX_PIN, estadoPin);
    tiempoAnterior = micros();
}

void actualizarTransmisionTexto() {
    unsigned long tiempoActual = micros();
    
    if (tiempoActual - tiempoAnterior >= (periodoActual_us / 2)) {
        tiempoAnterior = tiempoActual;
        
        estadoPin = !estadoPin;
        digitalWrite(TX_PIN, estadoPin);
        
        if (estadoPin == LOW) {
            cicloActual++;
            
            if (cicloActual >= numCiclosTotal) {
                bitIndex--;
                
                if (bitIndex < 0) {
                    Serial.println(" ✓");
                    digitalWrite(TX_PIN, LOW);

                    inicioGuarda_ms = millis();
                    estadoActual = GUARDA;
                } else {
                    iniciarTransmisionBitTexto();
                }
            }
        }
    }
}

// ==================== FUNCIONES AUDIO ====================

void iniciarTransmisionAudio(char comando) {
    // Elegir frecuencia portadora según el comando
    float frecuencia = (comando == '0') ? F_AUDIO_LOW : F_AUDIO_HIGH;

    Serial.print("TX AUDIO comando ");
    Serial.print(comando);
    Serial.print(" → ");
    Serial.print(frecuencia);
    Serial.println(" Hz");

    periodoActual_us = 1000000.0 / frecuencia;

    estadoPin = HIGH;
    digitalWrite(TX_PIN, estadoPin);
    tiempoAnterior = micros();

    inicioAudio_ms = millis();
    estadoActual = TRANSMITIENDO_AUDIO;
}

void actualizarTransmisionAudio() {
    unsigned long tiempoActual = micros();
    
    // Generar onda cuadrada a F_AUDIO_x
    if (tiempoActual - tiempoAnterior >= (periodoActual_us / 2)) {
        tiempoAnterior = tiempoActual;
        estadoPin = !estadoPin;
        digitalWrite(TX_PIN, estadoPin);
    }

    // Terminar ráfaga de audio después de T_AUDIO
    if (millis() - inicioAudio_ms >= (unsigned long)(T_AUDIO * 1000)) {
        digitalWrite(TX_PIN, LOW);
        inicioGuarda_ms = millis();
        estadoActual = GUARDA;
    }
}
