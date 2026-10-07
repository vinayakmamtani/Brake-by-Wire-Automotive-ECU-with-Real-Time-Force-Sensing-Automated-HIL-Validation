import serial
import time
import re
import sys

PORT = "COM3"
BAUD = 115200

LOG_REGEX = re.compile(r"RAW:\s*(-?\d+)\s*\|\s*NET:\s*(-?\d+)\s*\|\s*BRAKE:\s*(\d+)%(.*)")

def parse_telemetry(line):
    m = LOG_REGEX.search(line)
    if m:
        return {
            "raw": int(m.group(1)),
            "net": int(m.group(2)),
            "pct": int(m.group(3)),
            "fault": "[FAULTSAFE]" in m.group(4)
        }
    return None

def run_hil_suite():
    print("=" * 60)
    print(" AUTOMOTIVE HIL VALIDATION SUITE: BRAKE-BY-WIRE ECU")
    print("=" * 60)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=3)
    except Exception as e:
        print(f"Error opening port {PORT}: {e}")
        sys.exit(1)

    time.sleep(2) 
    ser.reset_input_buffer()

    #TEST CASE 01: Null Baseline Verification
    print("\n[RUNNING] TC-01: Zero-Load Baseline & Noise Plausibility (50 frames)...")
    tc1_passed = True
    samples = []
    
    while len(samples) < 50:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        data = parse_telemetry(line)
        if data:
            samples.append(data)
            if data["pct"] != 0 or data["fault"]:
                tc1_passed = False
                print(f"  VIOLATION: Non-zero demand at rest -> {data}")

    if tc1_passed:
        print("  -> TC-01 PASSED: Baseline is stable at 0% demand.")
    else:
        print("  -> TC-01 FAILED: Phantom braking detected.")

    # TEST CASE 02: Dynamic Stroke & Transfer Function
    print("\n[ACTION REQUIRED] TC-02: Transfer Function Test.")
    print(">>> Squeeze the load cell firmly within the next 5 seconds! <<<")
    
    start_time = time.time()
    max_pct_observed = 0
    monotonic = True
    last_pct = 0

    while time.time() - start_time < 6:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        data = parse_telemetry(line)
        if data:
            current_pct = data["pct"]
            if current_pct > max_pct_observed:
                max_pct_observed = current_pct
            print(f"  Live Stroke: {current_pct}% | Raw: {data['raw']}", end="\r")

    print(f"\n  -> Peak Commanded Brake Effort: {max_pct_observed}%")
    tc2_passed = max_pct_observed >= 70

    if tc2_passed:
        print("  -> TC-02 PASSED: Dynamic deflection registered successfully.")
    else:
        print("  -> TC-02 FAILED: Deflection did not reach threshold (press harder or check gain).")

    #TEST SUMMARY
    print("\n" + "=" * 60)
    print(f" HIL RESULT: TC-01: {'PASS' if tc1_passed else 'FAIL'} | TC-02: {'PASS' if tc2_passed else 'FAIL'}")
    print("=" * 60)
    ser.close()

if __name__ == "__main__":
    run_hil_suite()