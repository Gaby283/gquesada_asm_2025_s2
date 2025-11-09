/*
 * TRANSMISOR FSK TDM (un solo GPIO)
 * - Slot BAJO (360/540): piloto 0101 o paquete de AUDIO ('0'/'1')
 * - Slot ALTO (1400/1700): paquete de TEXTO (preambulo 0x55 + 1 char)
 *
 * TDM: [LOW slot] -> GUARD -> [HIGH slot] -> GUARD -> repeat
 * Forma de onda: cuadrada (simple), con tiempos por micros() (no delay()).
 */

#include <Arduino.h>

#define TX_PIN 25

// -------- Frecuencias (Hz) --------
// Canal BAJO (piloto/audio)
static const float F_LOW0 = 360.0f;   // bit 0
static const float F_LOW1 = 540.0f;   // bit 1
// Canal ALTO (texto)
static const float F_HIGH0 = 1400.0f; // bit 0
static const float F_HIGH1 = 1700.0f; // bit 1

// -------- Tiempos --------
// Duración EXACTA de la ventana FFT del RX: 512 / 4000 Hz = 128 ms
static const float TB_ms         = 128.0f; // no 120 ni 140
static const float GUARD_ms      = 90.0f;  // un poco más de 1/2 frame
static const float AUDIO_BURST_ms= 512.0f; // múltiplo del frame (4 frames)


// Preambulo para TEXTO (mejora sincronía en RX)
static const uint8_t PREAMBULO   = 0x55;   // 01010101
static const uint8_t PREAMB_REPS = 2;

// -------- Colas simples --------
#define QSZ 64
char qText[QSZ];   int qhT=0, qtT=0, qcT=0;   // cola para texto
char qAudio[QSZ];  int qhA=0, qtA=0, qcA=0;   // cola para audio ('0'/'1')

// ===== Utilidad de colas =====
bool qPush(char *q,int &qh,int &qt,int &qc,char v){ if(qc>=QSZ) return false; q[qt]=v; qt=(qt+1)%QSZ; qc++; return true; }
bool qPop (char *q,int &qh,int &qt,int &qc,char &v){ if(qc<=0)  return false; v=q[qh]; qh=(qh+1)%QSZ; qc--; return true; }

// ===== Generación de cuadrada =====
inline void startLineHigh(){ digitalWrite(TX_PIN, HIGH); }
inline void setLine(bool s){ digitalWrite(TX_PIN, s?HIGH:LOW); }
inline void stopLine(){ digitalWrite(TX_PIN, LOW); }

// cuadrada por duración (ms) a fHz
void sendToneBurst(float fHz, float dur_ms){
  if(fHz <= 0.0f || dur_ms <= 0.0f) return;
  const float T_us = 1000000.0f / fHz;     // periodo
  const float half_us = T_us / 2.0f;       // semiperiodo
  const unsigned long total_us = (unsigned long)(dur_ms * 1000.0f);

  unsigned long t0 = micros();
  bool state = true; setLine(state);
  unsigned long last = micros();
  while(micros() - t0 < total_us){
    unsigned long now = micros();
    if(now - last >= (unsigned long)half_us){
      state = !state; setLine(state);
      last = now;
    }
  }
  stopLine();
}

// bit FSK con par (f0,f1) y TB_ms (bloqueante)
void sendFSKBit(bool bit, float f0, float f1, float Tb_ms){
  const float f = bit ? f1 : f0;
  const float T_us = 1000000.0f / f;
  const float half_us = T_us / 2.0f;
  const unsigned long total_us = (unsigned long)(Tb_ms * 1000.0f);

  unsigned long t0 = micros();
  bool state = true; setLine(state);
  unsigned long last = micros();
  while(micros() - t0 < total_us){
    unsigned long now = micros();
    if(now - last >= (unsigned long)half_us){
      state = !state; setLine(state);
      last = now;
    }
  }
  stopLine();
}

// silencio (guard time)
void guardSilence(float ms){
  stopLine();
  unsigned long t0 = micros();
  const unsigned long total_us = (unsigned long)(ms * 1000.0f);
  while(micros() - t0 < total_us){ /* busy wait */ }
}

// ===== Paquetes (slots) =====

// SLOT BAJO: si hay audio en cola, envía ráfaga a LOW/HIGH;
// si no, envía piloto 0101 (4 bits) en (F_LOW0,F_LOW1)
void slotLOW(){
  char cmd;
  if(qPop(qAudio, qhA, qtA, qcA, cmd)){
    // '0' usa F_LOW0, '1' usa F_LOW1, como ráfaga continua
    float f = (cmd=='1') ? F_LOW1 : F_LOW0;
    Serial.print("[LOW] AUDIO cmd="); Serial.print(cmd);
    Serial.print(" f="); Serial.print(f); Serial.println(" Hz");
    sendToneBurst(f, AUDIO_BURST_ms);       // ráfaga
  }else{
    // Piloto 0101
    Serial.println("[LOW] PILOTO 0101");
    const bool patt[4] = {0,1,0,1};
    for(int i=0;i<4;i++) sendFSKBit(patt[i], F_LOW0, F_LOW1, TB_ms);
  }
}

// SLOT ALTO: si hay texto en cola, envía preámbulo + 1 carácter (MSB-first)
void slotHIGH(){
  char c;
  if(!qPop(qText, qhT, qtT, qcT, c)){
    Serial.println("[HIGH] (sin texto)");
    return;
  }

  Serial.print("[HIGH] TEXTO: '"); Serial.print(c); Serial.print("' bits: ");

  // preámbulo 0x55 repetido
  for(uint8_t r=0;r<PREAMB_REPS;r++){
    for(int b=7;b>=0;b--){
      bool bit = ( (PREAMBULO >> b) & 1 );
      sendFSKBit(bit, F_HIGH0, F_HIGH1, TB_ms);
    }
  }

  // carácter (MSB primero)
  for(int b=7;b>=0;b--){
    bool bit = ( ((uint8_t)c >> b) & 1 );
    Serial.print(bit);
    sendFSKBit(bit, F_HIGH0, F_HIGH1, TB_ms);
  }
  Serial.println(" ✓");
}

// ===== SETUP/LOOP =====
void setup(){
  Serial.begin(115200);
  delay(400);
  pinMode(TX_PIN, OUTPUT);
  stopLine();

  Serial.println("\n=== TX FSK TDM ===");
  Serial.println("LOW slot:  PILOTO(0101) o AUDIO ('0'/'1') @ 360/540 Hz");
  Serial.println("HIGH slot: TEXTO (preamb+char)           @ 1400/1700 Hz");
  Serial.println("TDM: [LOW] -> GUARD -> [HIGH] -> GUARD\n");
}

void loop(){
  // Leer entrada y encolar
  while(Serial.available()){
    char ch = (char)Serial.read();
    if(ch=='0' || ch=='1'){
      qPush(qAudio, qhA, qtA, qcA, ch);
    }else if((uint8_t)ch >= 32){
      qPush(qText, qhT, qtT, qcT, ch);
    }
  }

  // ---- SLOT BAJO ----
  slotLOW();
  guardSilence(GUARD_ms);

  // ---- SLOT ALTO ----
  slotHIGH();
  guardSilence(GUARD_ms);
}
