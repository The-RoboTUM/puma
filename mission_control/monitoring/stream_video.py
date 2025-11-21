import cv2
import numpy as np
import threading
import time

# Threaded capture to reduce latency by always keeping the latest frame
class RTSPCapture:
    def __init__(self, url):
        self.cap = cv2.VideoCapture(url)
        # Try to set buffer size to 1 (backend dependent)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.lock = threading.Lock()
        self.frame = None
        self.ret = False
        self.running = True
        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = True
        self.thread.start()

    def _reader(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                # If stream drops, wait a bit before retrying
                time.sleep(0.1)
                continue
            with self.lock:
                self.ret = ret
                self.frame = frame

    def read(self):
        with self.lock:
            return self.ret, self.frame

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.running = False
        self.thread.join()
        self.cap.release()

urls = [
    "rtsp://10.21.31.103:8554/video1",  # Front
    "rtsp://10.21.31.103:8554/video2",  # Rear
]
caps = [RTSPCapture(u) for u in urls]
for i, cap in enumerate(caps):
    if not cap.isOpened():
        print(f"Failed to open stream {i}: {urls[i]}")

TARGET_H = 480  # resize to same height for tiling

def read_resized(cap, target_h):
    ok, frame = cap.read()
    if not ok or frame is None:
        return None
    h, w = frame.shape[:2]
    scale = target_h / float(h)
    return cv2.resize(frame, (int(w * scale), target_h), interpolation=cv2.INTER_AREA)

while True:
    frames = [read_resized(c, TARGET_H) for c in caps]
    valid = [f for f in frames if f is not None]
    if valid:
        # pad missing streams with black
        max_w = max(f.shape[1] for f in valid)
        tiled = []
        for f in frames:
            if f is None:
                f = np.zeros((TARGET_H, max_w, 3), dtype=np.uint8)
            else:
                pad = max_w - f.shape[1]
                if pad > 0:
                    f = np.pad(f, ((0,0),(0,pad),(0,0)), mode='constant')
            tiled.append(f)
        view = np.hstack(tiled)
        cv2.imshow("Lynx M20 Cameras", view)

    if cv2.waitKey(1) == 27:  # ESC
        break

for c in caps:
    c.release()
cv2.destroyAllWindows()
