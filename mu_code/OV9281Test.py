import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# -----------------------------
# GPIO SETUP FOR LASER (GPIO 12)
# -----------------------------
LASER_PIN = 12
GPIO.setmode(GPIO.BCM)
GPIO.setup(LASER_PIN, GPIO.OUT)

# Turn laser ON (PNP open collector → HIGH = ON)
GPIO.output(LASER_PIN, GPIO.HIGH)
time.sleep(0.5)  # 500 ms warm-up

# -----------------------------
# CAMERA INIT
# -----------------------------
picam2 = Picamera2()

print("\n=== AVAILABLE RAW8 MODES ===")
modes = picam2.sensor_modes
raw8_modes = []

for m in modes:
    if m.get("bit_depth") == 8:
        raw8_modes.append(m)
        print(f"Mode: {m['size'][0]}x{m['size'][1]}  @ {m['fps']:.2f} FPS  (RAW8)")

if not raw8_modes:
    print("No RAW8 modes found!")
    exit(1)

print("=============================\n")

# -----------------------------
# SELECT RAW8 MODE
# -----------------------------
# You can change this to any mode printed above.
selected_size = (640, 400)   # ⭐ smallest RAW8 mode exposed by libcamera

print(f"Using RAW8 mode: {selected_size[0]}x{selected_size[1]}\n")

# -----------------------------
# CAMERA CONFIGURATION
# -----------------------------
config = picam2.create_video_configuration(
    raw={"format": "R8", "size": selected_size},
    buffer_count=6,
    controls={
        "FrameDurationLimits": (4761, 4761),  # ~210 FPS for 640×400
        "ExposureTime": 20,                   # 20 µs
        "AnalogueGain": 1.0,
        "AeEnable": False,
        "NoiseReductionMode": 0
    }
)

picam2.configure(config)
picam2.start()

# Re-apply controls AFTER start
picam2.set_controls({
    "AeEnable": False,
    "ExposureTime": 20,
    "AnalogueGain": 1.0,
    "NoiseReductionMode": 0
})

print("Starting 10‑frame stacking...\n")

# -----------------------------
# TIMING STATS
# -----------------------------
min_ms = 1e9
max_ms = 0
avg_ms = 0
count = 0

width, height = selected_size
acc_shape = (height, width)

# -----------------------------
# PERFORM EXACTLY 5 STACKS
# -----------------------------
for stack_index in range(5):

    # Flush buffer
    picam2.capture_array("raw")

    # ⭐ accumulator for selected mode
    accumulator = np.zeros(acc_shape, dtype=np.uint16)

    t_stack_start = time.perf_counter_ns()

    for i in range(10):
        t0 = time.perf_counter_ns()
        img = picam2.capture_array("raw").astype(np.uint16)
        t1 = time.perf_counter_ns()

        ms = (t1 - t0) / 1e6

        # Stats
        count += 1
        if ms < min_ms: min_ms = ms
        if ms > max_ms: max_ms = ms
        avg_ms += (ms - avg_ms) / count

        accumulator += img

    t_stack_end = time.perf_counter_ns()
    true_stack_ms = (t_stack_end - t_stack_start) / 1e6

    print(f"Stack {stack_index+1} time: {true_stack_ms:.3f} ms")

    # -----------------------------
    # AVERAGE THE STACK
    # -----------------------------
    averaged = (accumulator / 10).astype(np.uint8)

    # -----------------------------
    # SAVE AVERAGED IMAGE
    # -----------------------------
    img_name = f"stack_{stack_index+1}_{width}x{height}.png"
    cv2.imwrite(img_name, averaged)
    print(f"Saved {img_name}")

    # -----------------------------
    # HISTOGRAM
    # -----------------------------
    plt.figure(figsize=(10, 6))
    plt.hist(averaged.flatten(), bins=256, range=(0, 255),
             color='blue', alpha=0.7)
    plt.title(f"Histogram for stack {stack_index+1} ({width}×{height})")
    plt.xlabel("Pixel Intensity (0–255)")
    plt.ylabel("Count")
    plt.grid(True)

    hist_name = f"stack_{stack_index+1}_{width}x{height}_hist.png"
    plt.savefig(hist_name)
    plt.close()
    print(f"Saved {hist_name}")

# -----------------------------
# FINAL STATS
# -----------------------------
fps = 1000.0 / avg_ms if avg_ms > 0 else 0

print("\nFinal stats after 5 stacks:")
print(f"min: {min_ms:.3f} ms")
print(f"max: {max_ms:.3f} ms")
print(f"avg: {avg_ms:.3f} ms")
print(f"fps: {fps:.1f}")
print(f"frames captured: {count}")

# -----------------------------
# TURN LASER OFF
# -----------------------------
GPIO.output(LASER_PIN, GPIO.LOW)
GPIO.cleanup()

print("Laser OFF, done.")
