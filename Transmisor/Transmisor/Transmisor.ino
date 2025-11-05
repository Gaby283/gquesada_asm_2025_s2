/*
 * TRANSMISOR FSK DIGITAL - Proyecto CE 1110
 * ESP32 #1 - Genera onda cuadrada modulada FSK
 * 
 * Conexiones:
 * - GPIO25 → Cable a receptores (señal digital FSK)
 * - GND → GND común
 */

#include <Arduino.h>

// ==================== CONFIGURACIÓN ====================
#define TX_PIN 25           // Pin de salida digital
#define F1 800.0f           // Frecuencia bit 0 (Hz)
#define F2 1600.0f          // Frecuencia bit 1 (Hz)
#define Tb 0.01f            // Duración por bit (10ms)

// ==================== VARIABLES GLOBALES ====================
const float periodo_f1 = 1000000.0 / F1;  // Periodo en microsegundos (1250 μs)
const float periodo_f2 = 1000000.0 / F2;  // Periodo en microsegundos (625 μs)

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, LOW);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  TRANSMISOR FSK DIGITAL - ESP32       ║");
    Serial.println("║  CE 1110 - Proyecto Comunicación      ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    Serial.println("Configuración:");
    Serial.print("  • Pin TX: GPIO");
    Serial.println(TX_PIN);
    Serial.print("  • F1 (bit 0): ");
    Serial.print(F1, 0);
    Serial.println(" Hz");
    Serial.print("  • F2 (bit 1): ");
    Serial.print(F2, 0);
    Serial.println(" Hz");
    Serial.print("  • Duración bit (Tb): ");
    Serial.print(Tb * 1000, 1);
    Serial.println(" ms");
    
    Serial.println("\n✅ Sistema listo.");
    Serial.println("📝 Escribe texto para transmitir:\n");
}

// ==================== LOOP ====================
void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        
        // Ignorar caracteres de control
        if (c < 32 && c != '\n' && c != '\r') {
            return;
        }
        
        Serial.print("TX: '");
        Serial.print(c);
        Serial.print("' (ASCII ");
        Serial.print((int)c);
        Serial.print(") → ");
        
        transmitirCaracter(c);
        
        Serial.println(" ✓\n");
    }
}

// ==================== FUNCIONES ====================

void transmitirCaracter(char c) {
    int ascii = (int)c;
    
    // Transmitir 8 bits (MSB primero, como esperan los receptores)
    for (int i = 7; i >= 0; i--) {
        int bit = (ascii >> i) & 1;
        transmitirBitFSK(bit);
        Serial.print(bit);
    }
}

void transmitirBitFSK(int bit) {
    // Seleccionar frecuencia según bit
    float frecuencia = (bit == 0) ? F1 : F2;
    float periodo_us = 1000000.0 / frecuencia;  // Periodo en microsegundos
    
    // Calcular cuántos ciclos completos caben en Tb
    int num_ciclos = (int)(frecuencia * Tb);  // ej: 800*0.01 = 8 ciclos
    
    // Generar onda cuadrada por Tb segundos
    for (int ciclo = 0; ciclo < num_ciclos; ciclo++) {
        // Medio periodo HIGH
        digitalWrite(TX_PIN, HIGH);
        delayMicroseconds(periodo_us / 2);
        
        // Medio periodo LOW
        digitalWrite(TX_PIN, LOW);
        delayMicroseconds(periodo_us / 2);
    }
}
