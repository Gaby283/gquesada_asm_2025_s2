/*
 * TRANSMISOR FSK-TDM (VERSIÓN FINAL CORRECTA)
 * Ambos canales usan FSK
 */

#include <Arduino.h>

#define TX_PIN 25

// -------- Frecuencias FSK --------
// Canal BAJO (piloto en FSK)
static const float F_LOW0 = 360.0f;   // bit 0
static const float F_LOW1 = 540.0f;   // bit 1

// Canal ALTO (texto en FSK)
static const float F_HIGH0 = 800.0f;  // bit 0
static const float F_HIGH1 = 1600.0f; // bit 1

// -------- Configuración del Piloto --------
static const float PILOTO_FREQ = 450.0f;  // ← Frecuencia objetivo (300-800 Hz)
static const int PILOTO_BITS = 8;         // Bits por slot de piloto

// -------- Tiempos --------
static const float TB_ms = 140.0f;
static const float GUARD_ms = 200.0f;

// -------- Preámbulo --------
static const uint8_t PREAMBULO = 0x55;

// -------- Cola de texto --------
#define QSZ 64
char qText[QSZ];
int qhT = 0, qtT = 0, qcT = 0;

bool qPush(char *q, int &qh, int &qt, int &qc, char v) {
  if (qc >= QSZ) return false;
  q[qt] = v;
  qt = (qt + 1) % QSZ;
  qc++;
  return true;
}

bool qPop(char *q, int &qh, int &qt, int &qc, char &v) {
  if (qc <= 0) return false;
  v = q[qh];
  qh = (qh + 1) % QSZ;
  qc--;
  return true;
}

// ===== Generación de onda =====
inline void setLine(bool s) { 
  digitalWrite(TX_PIN, s ? HIGH : LOW); 
}

inline void stopLine() { 
  digitalWrite(TX_PIN, LOW); 
}

void sendFSKBit(bool bit, float f0, float f1, float Tb_ms) {
  const float f = bit ? f1 : f0;
  const float T_us = 1000000.0f / f;
  const float half_us = T_us / 2.0f;
  const unsigned long total_us = (unsigned long)(Tb_ms * 1000.0f);

  unsigned long t0 = micros();
  bool state = true;
  setLine(state);
  unsigned long last = micros();
  
  while (micros() - t0 < total_us) {
    unsigned long now = micros();
    if (now - last >= (unsigned long)half_us) {
      state = !state;
      setLine(state);
      last = now;
    }
  }
  stopLine();
}

void guardSilence(float ms) {
  stopLine();
  unsigned long t0 = micros();
  const unsigned long total_us = (unsigned long)(ms * 1000.0f);
  while (micros() - t0 < total_us) { }
}

// ===== SLOT BAJO: Piloto en FSK =====
void slotLOW() {
    // Calcular cuántos '1' necesitamos
    float ratio = (PILOTO_FREQ - F_LOW0) / (F_LOW1 - F_LOW0);
    ratio = constrain(ratio, 0.0, 1.0);
    
    int numUnos = (int)(ratio * PILOTO_BITS + 0.5);
    
    /*Serial.print("[PILOTO] ");
    Serial.print(PILOTO_FREQ);
    Serial.print(" Hz → Patrón alternado (");
    Serial.print(numUnos);
    Serial.print("/");
    Serial.print(PILOTO_BITS);
    Serial.print("): ");
    */
    // NUEVO: Generar patrón ALTERNADO
    int unos_restantes = numUnos;
    int ceros_restantes = PILOTO_BITS - numUnos;
    
    for (int i = 0; i < PILOTO_BITS; i++) {
        bool bit;
        
        // Estrategia: Alternar lo más posible
        if (i % 2 == 0) {
            // Posición par: preferir '0'
            if (ceros_restantes > 0) {
                bit = false;
                ceros_restantes--;
            } else {
                bit = true;
                unos_restantes--;
            }
        } else {
            // Posición impar: preferir '1'
            if (unos_restantes > 0) {
                bit = true;
                unos_restantes--;
            } else {
                bit = false;
                ceros_restantes--;
            }
        }
        
        //Serial.print(bit);
        sendFSKBit(bit, F_LOW0, F_LOW1, TB_ms);
    }
    //Serial.println();
}


// ===== SLOT ALTO: Texto en FSK =====
void slotHIGH() {
  char c;
  if (!qPop(qText, qhT, qtT, qcT, c)) {
    return;
  }

  Serial.print("[TEXTO] '");
  Serial.print(c);
  Serial.print("' bits: ");

  // Preámbulo
  for (int b = 7; b >= 0; b--) {
    bool bit = ((PREAMBULO >> b) & 1);
    sendFSKBit(bit, F_HIGH0, F_HIGH1, TB_ms);
  }

  // Carácter
  for (int b = 7; b >= 0; b--) {
    bool bit = (((uint8_t)c >> b) & 1);
    Serial.print(bit);
    sendFSKBit(bit, F_HIGH0, F_HIGH1, TB_ms);
  }
  Serial.println(" ✓");
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(400);
  pinMode(TX_PIN, OUTPUT);
  stopLine();

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║        TRANSMISOR FSK-TDM                      ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.println();
  
  Serial.print("📡 SLOT BAJO  (Piloto FSK): ");
  Serial.print(PILOTO_FREQ);
  Serial.print(" Hz (usando ");
  Serial.print(F_LOW0);
  Serial.print("/");
  Serial.print(F_LOW1);
  Serial.println(" Hz)");
  
  Serial.print("📝 SLOT ALTO  (Texto FSK):  ");
  Serial.print(F_HIGH0);
  Serial.print("/");
  Serial.print(F_HIGH1);
  Serial.println(" Hz");
  
  Serial.println();
  Serial.println("✅ Rango válido piloto: 360-540 Hz");
  Serial.println("✅ Ambos canales usan modulación FSK");
  Serial.println("✅ Escribe texto por Serial...");
  Serial.println();
}

// ===== LOOP =====
void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if ((uint8_t)ch >= 32) {
      qPush(qText, qhT, qtT, qcT, ch);
    }
  }

  slotLOW();
  guardSilence(GUARD_ms);

  slotHIGH();
  guardSilence(GUARD_ms);
}
