import cv2
import numpy as np

FRONT = "rtsp://10.21.31.103:8554/video1"
REAR  = "rtsp://10.21.31.103:8554/video2"

def gst_rtsp(url, tcp=False):
    proto = "tcp" if tcp else "udp"
    return (
        f"rtspsrc location={url} latency=0 protocols={proto} ! "
        f"rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        f"appsink sync=false drop=true max-buffers=1"
    )

cap1 = cv2.VideoCapture(gst_rtsp(FRONT, tcp=False), cv2.CAP_GSTREAMER)
cap2 = cv2.VideoCapture(gst_rtsp(REAR,  tcp=False), cv2.CAP_GSTREAMER)
if not cap1.isOpened(): raise SystemExit(f"Failed to open FRONT: {FRONT}")
if not cap2.isOpened(): raise SystemExit(f"Failed to open REAR:  {REAR}")

try:
    while True:
        ok1, f1 = cap1.read()
        ok2, f2 = cap2.read()

        if not ok1 and not ok2:
            if cv2.waitKey(1) == 27: break
            continue

        if ok1 and not ok2:
            f2 = np.zeros_like(f1)
        if ok2 and not ok1:
            f1 = np.zeros_like(f2)

        if ok1 and ok2 and f1.shape[0] != f2.shape[0]:
            h = max(f1.shape[0], f2.shape[0])
            def pad_to_h(img, H):
                pad = H - img.shape[0]
                if pad <= 0: return img
                return np.pad(img, ((0,pad),(0,0),(0,0)), mode="constant")
            f1, f2 = pad_to_h(f1, h), pad_to_h(f2, h)

        if ok1 or ok2:
            view = np.hstack([f1, f2])
            cv2.imshow("Lynx M20 Cameras (GStreamer)", view)

        if cv2.waitKey(1) == 27:  # ESC
            break
finally:
    cap1.release(); cap2.release()
    cv2.destroyAllWindows()
