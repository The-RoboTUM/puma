import cv2

for url in ["rtsp://10.21.31.103:8554/video1", "rtsp://10.21.31.103:8554/video2"]:
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print("Failed to open:", url)
        continue
    while True:
        ok, frame = cap.read()
        if not ok: break
        cv2.imshow(url, frame)
        if cv2.waitKey(1) == 27:  # ESC to quit
            break
    cap.release()
cv2.destroyAllWindows()
