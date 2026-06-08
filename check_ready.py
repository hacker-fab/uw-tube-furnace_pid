"""
Run this before starting the main program.
Takes 30s of temperature samples and checks if the furnace is stable enough to calibrate.
"""
import time
import sys
import serial

PORT = "/dev/cu.usbserial-A9XUD4EB"
BAUD = 9600
SAMPLES = 30          # seconds
SAMPLE_TIME = 1.0

# Thresholds — based on a clean run (0.10°C, 1.36°C/min)
MAX_TEMP_NOISE  = 1.0   # °C peak-to-peak
MAX_RATE_NOISE  = 5.0   # °C/min max absolute rate


def read_pv(ser):
    payload = "010310000001"
    byte_sum = sum(int(payload[i:i+2], 16) for i in range(0, len(payload), 2))
    lrc = f"{(0x100 - (byte_sum & 0xFF)) & 0xFF:02X}"
    ser.write(f":{payload}{lrc}\r\n".encode("ascii"))
    time.sleep(0.1)
    resp = ser.read(100)
    if not resp or not resp.startswith(b":") or len(resp) < 11:
        return None
    hex_val = resp[7:11].decode()
    val = int(hex_val, 16)
    if hex_val.upper() in ("8002", "8003", "8004", "8006", "8007"):
        return None
    return val / 10.0


def calculate_rate(times, temps):
    if len(temps) < 2:
        return 0.0
    window = min(len(temps), 3)
    xs = times[-window:]
    ys = temps[-window:]
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    denom = sum((x - mx) ** 2 for x in xs)
    if denom == 0:
        return 0.0
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom
    return slope * 60  # °C/min


def main():
    print(f"Connecting to {PORT}...")
    try:
        ser = serial.Serial(PORT, BAUD, bytesize=7, parity="E", stopbits=1, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    print(f"Sampling for {SAMPLES}s — do not start the main program yet.\n")

    temps, times, rates = [], [], []
    start = time.time()

    for i in range(SAMPLES):
        target = start + i * SAMPLE_TIME
        while time.time() < target:
            time.sleep(0.001)

        t = read_pv(ser)
        if t is None:
            print(f"  [{i+1:02d}/{SAMPLES}] Read failed, skipping")
            continue

        elapsed = time.time() - start
        times.append(elapsed)
        temps.append(t)
        rate = calculate_rate(times, temps)
        rates.append(rate)

        print(f"  [{i+1:02d}/{SAMPLES}] {t:.1f}°C  {rate:+.2f}°C/min", end="\r")

    ser.close()
    print()

    if len(temps) < 10:
        print("\nERROR: Too few successful reads. Check serial connection.")
        sys.exit(1)

    temp_noise = max(temps) - min(temps)        # peak-to-peak
    rate_noise = max(abs(r) for r in rates)     # max absolute rate

    # What the main program would set
    projected_hys           = temp_noise * 3.0
    projected_noise_ceiling = rate_noise * 3.0

    print(f"\n--- Results ---")
    print(f"Current temp   : {temps[-1]:.1f}°C")
    print(f"Temp noise     : {temp_noise:.2f}°C   (limit: < {MAX_TEMP_NOISE}°C)")
    print(f"Rate noise     : {rate_noise:.2f}°C/min  (limit: < {MAX_RATE_NOISE}°C/min)")
    print(f"Projected hysteresis    : {projected_hys:.2f}°C")
    print(f"Projected noise ceiling : {projected_noise_ceiling:.2f}°C/min")

    temp_ok = temp_noise  < MAX_TEMP_NOISE
    rate_ok = rate_noise  < MAX_RATE_NOISE

    print()
    if temp_ok and rate_ok:
        print("READY — furnace is stable, safe to start the main program.")
        sys.exit(0)
    else:
        reasons = []
        if not temp_ok:
            reasons.append(f"temp swing {temp_noise:.2f}°C > {MAX_TEMP_NOISE}°C")
        if not rate_ok:
            reasons.append(f"drift rate {rate_noise:.2f}°C/min > {MAX_RATE_NOISE}°C/min")
        print(f"NOT READY — {'; '.join(reasons)}. Wait and try again.")
        sys.exit(1)


if __name__ == "__main__":
    main()
