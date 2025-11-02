
const int FREQ0 = 1000;   // frecuencia para bit 0
const int FREQ1 = 1200;   // frecuencia para bit 1
const unsigned long BIT_DURATION_MS = 200;  // duración de cada bit

int bits[] = {0, 1, 0, 1, 1, 0, 0};
const int numBits = sizeof(bits) / sizeof(bits[0]);

unsigned long lastBitChange = 0;
int currentBit = 0;
bool transmitting = true;

void setup() {
  Serial.begin(115200);
  Serial.println("Frecuencia mostrada según el bit actual...");
  lastBitChange = millis();
}

void loop() {
  if (transmitting) {
    unsigned long now = millis();

    // si ya pasó el tiempo del bit actual, avanza
    if (now - lastBitChange >= BIT_DURATION_MS) {
      currentBit++;
      lastBitChange = now;

      if (currentBit >= numBits) {
        transmitting = false;
        Serial.println("Transmisión finalizada.");
        return;
      }
    }

    // selecciona frecuencia según el bit actual
    int f = bits[currentBit] ? FREQ1 : FREQ0;

    Serial.printf("Bit %d -> Frecuencia: %d Hz\n", currentBit, f);

    unsigned long t0 = micros();
    while (micros() - t0 < 50000) {
      // espera 50 ms aprox sin bloquear
      yield(); // permite multitarea interna del ESP
    }
  }
}
