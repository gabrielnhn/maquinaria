import cv2
from datetime import datetime

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 90)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

sec = datetime.now().second
count = 0

retval, frame = cap.read()

while retval:
    count += 1
    if sec != datetime.now().second:
        print(f"FPS: {count}")
        count = 0
        sec = datetime.now().second
    
    cv2.imshow("bruh", frame)
    cv2.waitKey(3)
    
    retval, frame = cap.read()

