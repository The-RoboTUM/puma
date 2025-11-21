import cv2
import numpy as np
import threading
import time
import os

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "loglevel;quiet"

class RTSPCapture:
    def __init__(self, url):
        self.url = url
        self.cap = None
        self.lock = threading.Lock()
        self.frame = None
        self.ret = False
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                self.cap = cv2.VideoCapture(self.url)
                if not self.cap.isOpened():
                    time.sleep(2.0)
                    continue
                # Try to set buffer size to 1 (backend dependent)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            ret, frame = self.cap.read()
            if not ret:
                if self.cap:
                    self.cap.release()
                self.cap = None
                time.sleep(0.5)
                continue
            
            with self.lock:
                self.ret = ret
                self.frame = frame

    def read(self):
        with self.lock:
            return self.ret, self.frame

    def isOpened(self):
        return True # Always return true as we retry in background

    def release(self):
        self.running = False
        self.thread.join()
        if self.cap:
            self.cap.release()

class VideoStreamer:
    def __init__(self, urls, target_height=480):
        self.urls = urls
        self.target_height = target_height
        self.caps = []
        self.running = False

    def start(self):
        self.caps = [RTSPCapture(u) for u in self.urls]
        self.running = True

    def stop(self):
        self.running = False
        for c in self.caps:
            c.release()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    def _read_resized(self, cap):
        ok, frame = cap.read()
        if not ok or frame is None:
            return None
        h, w = frame.shape[:2]
        scale = self.target_height / float(h)
        return cv2.resize(frame, (int(w * scale), self.target_height), interpolation=cv2.INTER_AREA)

    def get_tiled_frame(self):
        frames = [self._read_resized(c) for c in self.caps]
        valid = [f for f in frames if f is not None]
        
        if not valid:
            return None

        # Pad missing streams with black to match width
        max_w = max(f.shape[1] for f in valid)
        tiled = []
        for f in frames:
            if f is None:
                f = np.zeros((self.target_height, max_w, 3), dtype=np.uint8)
            else:
                pad = max_w - f.shape[1]
                if pad > 0:
                    f = np.pad(f, ((0,0),(0,pad),(0,0)), mode='constant')
            tiled.append(f)
        return np.hstack(tiled)

    def run_display(self, window_name="Lynx M20 Cameras"):
        print(f"Video stream started. Press 'q' or 'ESC' in the video window to exit.")
        while self.running:
            view = self.get_tiled_frame()
            if view is not None:
                try:
                    cv2.imshow(window_name, view)
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27 or key == ord('q'):
                        break
                except cv2.error:
                    print("Error: OpenCV GUI functions not available (headless mode).")
                    print("Please run the Dashboard to view the video stream.")
                    break
        self.stop()

def main():
    urls = [
        "rtsp://10.21.31.103:8554/video1",  # Front
        "rtsp://10.21.31.103:8554/video2",  # Rear
    ]
    streamer = VideoStreamer(urls)
    try:
        streamer.start()
        streamer.run_display()
    except KeyboardInterrupt:
        pass
    finally:
        streamer.stop()

if __name__ == "__main__":
    main()
