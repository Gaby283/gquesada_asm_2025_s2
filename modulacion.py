import numpy as np
import matplotlib.pyplot as plt
import soundfile as sf
import subprocess
import os
from scipy.signal import resample

def modulacion(archivo="400hz.mp3"):
    """
    modulacion(archivo)
    Lee un archivo de audio (mp3 o wav), lo convierte a una secuencia de bits
    por ventanas de duración Tb y aplica FSK (f1 para 0, f2 para 1).

    Ejemplo: modulacion('Sonidos de prueba/400hz.mp3')
    """

    # --- 0. Verificar si el archivo existe ---
    if not os.path.exists(archivo):
        raise FileNotFoundError(f"No se encuentra el archivo: {archivo}")

    # --- 1. Intentar leer el archivo, convertir si es necesario ---
    base, ext = os.path.splitext(archivo)
    temp_wav = None

    try:
        x, Fs = sf.read(archivo)
    except Exception as e:
        # Intentar convertir con ffmpeg
        temp_wav = base + "_temp.wav"
        cmd = ["ffmpeg", "-y", "-i", archivo, "-ar", "44100", "-ac", "1", temp_wav, "-loglevel", "error"]
        try:
            result = subprocess.run(cmd, check=True, capture_output=True)
            x, Fs = sf.read(temp_wav)
        except (subprocess.CalledProcessError, FileNotFoundError):
            raise RuntimeError(f"No se pudo leer '{archivo}'. Instala ffmpeg o usa archivos WAV.")

    # --- 1b. Convertir a mono si es estéreo ---
    if x.ndim > 1:
        x = np.mean(x, axis=1)

    # --- 1c. Re-muestrear a 8 kHz ---
    Fs_target = 8000
    if Fs != Fs_target:
        n_new = int(len(x) * Fs_target / Fs)
        x = resample(x, n_new)
        Fs = Fs_target

    # --- 2. Parámetros FSK y framing ---
    Tb = 0.01  # duración del bit (10 ms)
    N = int(round(Tb * Fs))
    if N < 1:
        raise ValueError("Tb demasiado pequeño para la frecuencia de muestreo.")

    numBits = len(x) // N
    if numBits < 1:
        raise ValueError(f"Señal demasiado corta para formar al menos 1 bit con Tb = {Tb:.4f}s")

    # Generar bits por ventana (media positiva = 1)
    bits = np.zeros(numBits, dtype=bool)
    for k in range(numBits):
        seg = x[k * N:(k + 1) * N]
        bits[k] = np.mean(seg) > 0

    # --- 3. Portadoras FSK ---
    f1 = 800   # frecuencia para bit 0
    f2 = 1600  # frecuencia para bit 1
    n = np.arange(N)
    portadora1 = np.cos(2 * np.pi * f1 * n / Fs)
    portadora2 = np.cos(2 * np.pi * f2 * n / Fs)

    # --- 4. Modulación FSK (vectorizada) ---
    # modulada = kron(1-bits, portadora1) + kron(bits, portadora2)
    modulada = (np.kron(1 - bits.astype(float), portadora1) + 
                np.kron(bits.astype(float), portadora2))
    t_mod = np.arange(len(modulada)) / Fs

    # --- 5. Demodulación por correlación ---
    demod_bits = np.zeros(numBits, dtype=bool)
    for k in range(numBits):
        seg = modulada[k * N:(k + 1) * N]
        c1 = np.sum(seg * portadora1)
        c2 = np.sum(seg * portadora2)
        demod_bits[k] = c2 > c1

    # --- 6. Reconstrucción de señal demodulada ---
    demodulada_short = (demod_bits.astype(float) * 2 - 1)  # -1 o +1
    demodulada = np.repeat(demodulada_short, N)
    
    # Ajustar largo para comparar con x
    Lx = len(x)
    if len(demodulada) >= Lx:
        demodulada = demodulada[:Lx]
    else:
        demodulada = np.pad(demodulada, (0, Lx - len(demodulada)))

    t = np.arange(Lx) / Fs

    # --- 7. Gráficas ---
    plt.figure("Modulación FSK", figsize=(10, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(t[:min(1000, len(t))], x[:min(1000, len(x))])
    plt.title("Señal original")
    plt.xlabel("t (s)")
    plt.grid(True, alpha=0.3)

    plt.subplot(3, 1, 2)
    plt.plot(t_mod[:min(2000, len(t_mod))], modulada[:min(2000, len(modulada))])
    plt.title("Señal modulada FSK")
    plt.xlabel("t (s)")
    plt.grid(True, alpha=0.3)

    plt.subplot(3, 1, 3)
    plt.plot(t[:min(1000, len(t))], demodulada[:min(1000, len(demodulada))])
    plt.title("Señal demodulada")
    plt.xlabel("t (s)")
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

    # --- 8. FFT de la señal modulada ---
    print("Mostrando FFT de la señal modulada...")
    Y = np.fft.fft(modulada)
    f = np.fft.fftfreq(len(Y), 1 / Fs)
    
    plt.figure("FFT FSK", figsize=(10, 5))
    plt.plot(f[:len(f)//2], np.abs(Y[:len(Y)//2]))
    plt.title("FFT de la señal FSK")
    plt.xlabel("Frecuencia (Hz)")
    plt.ylabel("|X(f)|")
    plt.grid(True, alpha=0.3)
    plt.show()

    # --- 9. Limpiar archivos temporales ---
    if temp_wav and os.path.exists(temp_wav):
        try:
            os.remove(temp_wav)
        except:
            pass  # no crítico

    print(f"\nProcesamiento completo:")
    print(f"  - Bits generados: {numBits}")
    print(f"  - Frecuencia de muestreo: {Fs} Hz")
    print(f"  - Duración por bit: {Tb} s")

    print("\n🔧 Generando arrays para ESP32...")
    exportar_arrays_esp32(modulada, Fs, bits, muestras=512)
    
    return modulada, Fs, bits  # Agregar return


def exportar_arrays_esp32(modulada, Fs, bits, muestras=512):    
    # ========== Generar tonos PUROS ==========
    t = np.linspace(0, muestras/Fs, muestras, endpoint=False)
    
    # Tono puro de 800 Hz (bit 0)
    tono_800 = np.sin(2 * np.pi * 800 * t)
    
    # Tono puro de 1600 Hz (bit 1)
    tono_1600 = np.sin(2 * np.pi * 1600 * t)
    
    # Señal mixta: primeros 256 = 800Hz, últimos 256 = 1600Hz
    senal_mixta = np.concatenate([
        np.sin(2 * np.pi * 800 * t[:256]),
        np.sin(2 * np.pi * 1600 * t[:256])
    ])
    
    # ========== Convertir a valores DIGITALES (0 o 1) ==========
    def a_valores_digitales(senal):
        # 1 si la señal > 0, 0 si <= 0
        return (senal > 0).astype(int)
    
    dig_bit0 = a_valores_digitales(tono_800)
    dig_bit1 = a_valores_digitales(tono_1600)
    dig_mixta = a_valores_digitales(senal_mixta)
    
    # ========== Generar archivo C ==========
    with open("arrays_esp32.txt", 'w', encoding='utf-8') as f:
        f.write("/*\n")
        f.write(" * Arrays generados desde modulacion.py (DIGITAL)\n")
        f.write(" * Señales FSK binarias para ESP32\n")
        f.write(" * Cada muestra es 0 o 1 (sin escala ADC)\n")
        f.write(" */\n\n")
        
        f.write(f"// Parametros:\n")
        f.write(f"// - Frecuencia de muestreo: {Fs} Hz\n")
        f.write(f"// - Muestras por array: {muestras}\n")
        f.write(f"// - f1 = 800 Hz (bit 0), f2 = 1600 Hz (bit 1)\n\n")
        
        f.write("// ========== SENAL BIT 0 (800 Hz DIGITAL) ==========\n")
        f.write(generar_codigo_c("senal_bit0", dig_bit0))
        f.write("\n\n")
        
        f.write("// ========== SENAL BIT 1 (1600 Hz DIGITAL) ==========\n")
        f.write(generar_codigo_c("senal_bit1", dig_bit1))
        f.write("\n\n")
        
        f.write("// ========== SENAL MIXTA (800Hz + 1600Hz DIGITAL) ==========\n")
        f.write(generar_codigo_c("senal_mixta", dig_mixta))
    
    print(f"\n{'='*70}")
    print(f"✅ ARRAYS DIGITALES EXPORTADOS EXITOSAMENTE")
    print(f"{'='*70}")
    print(f"Archivo generado: arrays_esp32.txt")
    print(f"\nInformacion de los arrays:")
    print(f"  • senal_bit0: Tono digital de 800 Hz")
    print(f"  • senal_bit1: Tono digital de 1600 Hz")
    print(f"  • senal_mixta: 256 muestras de 800Hz + 256 de 1600Hz")
    print(f"  • Muestras: {muestras}")
    print(f"  • Frecuencia: {Fs} Hz")
    print(f"  • Duracion: {muestras/Fs:.3f} segundos")
    print(f"{'='*70}\n")
    
    return dig_bit0, dig_bit1, dig_mixta


def generar_codigo_c(nombre, datos, valores_por_linea=16):
    """
    Genera código C formateado para arrays
    """
    codigo = f"const int {nombre}[{len(datos)}] = {{\n"
    
    for i in range(0, len(datos), valores_por_linea):
        linea = datos[i:i+valores_por_linea]
        codigo += "    "
        codigo += ", ".join(f"{val:4d}" for val in linea)
        if i + valores_por_linea < len(datos):
            codigo += ","
        codigo += "\n"
    
    codigo += "};\n"
    return codigo



# Ejemplo de uso:
if __name__ == "__main__":
    modulacion("Sonidos de prueba/400hz.mp3")
