from ultralytics import YOLO
import numpy as np
import time

model = YOLO("best_ncnn_model")

# 640x480 blank frame, same size as the real camera
frame = np.zeros((480, 640, 3), dtype=np.uint8)

model.predict(source=frame, verbose=False)   # warm-up, untimed

times = []
for _ in range(10):
    t0 = time.time()
    model.predict(source=frame, conf=0.25, verbose=False)
    times.append((time.time() - t0) * 1000)

print(f"mean: {sum(times)/len(times):.0f} ms   min: {min(times):.0f}   max: {max(times):.0f}")

results = model.predict(source="test_rov.jpg", conf=0.25)
print("boxes found:", len(results[0].boxes))
for box in results[0].boxes:
    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
    print("center x:", (x1 + x2) / 2.0, "conf:", float(box.conf[0]))