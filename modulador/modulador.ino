// modulador_fft.ino
// ESP32: generador FSK 4-niveles + muestreo ADC + FFT -> muestra frecuencia detectada por Serial
// Requiere librería arduinoFFT (instalar desde Library Manager)

#include <arduinoFFT.h>
#include <Arduino.h>
#include <math.h> 

// ---------- PINES ----------
const int OUT_PIN = 25; 	// salida modulada (GPIO25)
const int ADC_PIN 	= 34; 	// entrada ADC para muestreo (GPIO34, ADC1_CH6)

// ---------- FRECUENCIAS BASE (Hz) ----------
// Estas frecuencias son las portadoras FSK
const int f00 = 100; 	// combinación 00
const int f01 = 200; 	// combinación 01
const int f10 = 300; 	// combinación 10
const int f11 = 400; 	// combinación 11

// ---------- MODULACIÓN ----------
volatile int bitA = 1; 
volatile int bitB = 0;
volatile int f_modulada = 1000; 

// ---------- Generador (toggle por software) ----------
volatile unsigned long halfPeriodUs = 500; 
volatile bool outputState = false;

// ---------- Parámetros de muestreo y FFT ----------
const uint16_t SAMPLES = 256; 	 	// potencia de 2
const double SAMPLING_FREQ = 8192.0; 	// Hz (frecuencia de muestreo)
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT FFT = ArduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// ---------- Control de tiempos de bits (cambio de combinación) ----------
const unsigned long changeIntervalMs = 2000; 
unsigned long lastChangeMs = 0;

// ---------- Protos ----------
void updateModulatedFrequency();
void generatorTask(void *pvParameters);
void samplingTask(void *pvParameters);
void setHalfPeriodFromFmod(int f);

void setup() {
	Serial.begin(115200);
	delay(200);
	pinMode(OUT_PIN, OUTPUT);
	digitalWrite(OUT_PIN, LOW);
	pinMode(ADC_PIN, INPUT);

	updateModulatedFrequency();

	xTaskCreatePinnedToCore(generatorTask, "GEN_TASK", 2048, NULL, 2, NULL, 1); 	// Core 1
	xTaskCreatePinnedToCore(samplingTask, "SAMP_TASK", 8192, NULL, 1, NULL, 0); 	// Core 0

	Serial.println("modulador_fft: arrancó. Espera información por Serial a 115200.");
	Serial.println("---------------------------------------------------------------");
	Serial.println("Modulacion FSK de 4 niveles (4-FSK) activa con simulacion de FFT.");
	Serial.println("---------------------------------------------------------------");
}

// --------------------------------------------------------------------------------

void generatorTask(void *pvParameters) {
	(void) pvParameters;
	unsigned long lastToggle = micros();

	while (true) {
		unsigned long now = micros();
		if ((unsigned long)(now - lastToggle) >= (unsigned long)halfPeriodUs) {
			outputState = !outputState;
			digitalWrite(OUT_PIN, outputState ? HIGH : LOW);
			lastToggle = now;
		}

		unsigned long nowMs = millis();
		if (nowMs - lastChangeMs >= changeIntervalMs) {
			lastChangeMs = nowMs;
			static int state = 0;
			state = (state + 1) % 4;
			switch (state) {
				case 0: bitA = 0; bitB = 0; break; 
				case 1: bitA = 0; bitB = 1; break; 
				case 2: bitA = 1; bitB = 0; break; 
				case 3: bitA = 1; bitB = 1; break; 
			}
			updateModulatedFrequency(); 
			Serial.print("[GEN] Nueva combinacion: bitA="); Serial.print(bitA);
			Serial.print(" bitB="); Serial.print(bitB);
			Serial.print(" -> f_modulada="); Serial.print(f_modulada);
			Serial.println(" Hz");
		}

		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

// --------------------------------------------------------------------------------

//////////////////////////////////////////////////////////
/// Task: SIMULA la captura de una señal cuadrada, ejecuta FFT y reporta
//////////////////////////////////////////////////////////
void samplingTask(void *pvParameters) {
	(void) pvParameters;

	while (true) {
		// --- SIMULACIÓN DE ONDA CUADRADA CON FRECUENCIA f_modulada ---
		
		double samplesPerHalfPeriod = (SAMPLING_FREQ / (double)f_modulada) / 2.0;
		
		const double LOW_LEVEL = 0.0;
		const double HIGH_LEVEL = 4095.0; 

		unsigned long t0 = micros(); 
		for (uint16_t i = 0; i < SAMPLES; i++) {
			
			if (samplesPerHalfPeriod < 1.0) { 
				vReal[i] = LOW_LEVEL; 
			} 
			else if (fmod((double)i, 2.0 * samplesPerHalfPeriod) < samplesPerHalfPeriod) {
				vReal[i] = HIGH_LEVEL;
			} else {
				vReal[i] = LOW_LEVEL;
			}
			vImag[i] = 0.0;
		}
		unsigned long t1 = micros();

		// --- Procesar FFT ---
		FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
		FFT.compute(FFT_FORWARD);
		FFT.complexToMagnitude();

		// Obtener pico dominante (Hz)
		double peak = FFT.majorPeak();

		// También calcular pico manualmente (corregido: usando vReal en lugar de vVal)
		uint16_t indexMax = 1; 
		double maxVal = vReal[1];
		for (uint16_t i = 2; i < SAMPLES/2; i++) {
			if (vReal[i] > maxVal) { maxVal = vReal[i]; indexMax = i; } // <<-- CORRECCIÓN APLICADA AQUÍ
		}
		double freqDetected = (double)indexMax * (SAMPLING_FREQ / (double)SAMPLES);

		// Imprimir resultados
		Serial.print("[FFT SIM] Detected peak(arduinoFFT) = ");
		Serial.print(peak, 1);
		Serial.print(" Hz, peak by bin = ");
		Serial.print(freqDetected, 1);
		Serial.print(" Hz, f_modulada(objetivo) = ");
		Serial.print(f_modulada);
		Serial.print(" Hz, calcTime(ms) = ");
		Serial.println((t1 - t0) / 1000.0, 3); 

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

// --------------------------------------------------------------------------------

void updateModulatedFrequency() {
	int baseFreq = 0;
	if (bitA == 0 && bitB == 0) baseFreq = f00;
	else if (bitA == 0 && bitB == 1) baseFreq = f01;
	else if (bitA == 1 && bitB == 0) baseFreq = f10;
	else if (bitA == 1 && bitB == 1) baseFreq = f11;

	f_modulada = baseFreq * 10;

	if (f_modulada <= 0) f_modulada = 1;
	setHalfPeriodFromFmod(f_modulada);
}

void setHalfPeriodFromFmod(int f) {
	unsigned long half = 500000UL / (unsigned long)f;
	if (half < 1) half = 1;
	halfPeriodUs = half;
}

void loop() {
	vTaskDelay(portMAX_DELAY);
}


