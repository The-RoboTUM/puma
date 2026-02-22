import cv2

url = "rtsp://admin:puma2126@192.168.1.108:554/stream1"

cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Failed to open stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("No frame")
        break

    cv2.imshow("PTZ Camera", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()