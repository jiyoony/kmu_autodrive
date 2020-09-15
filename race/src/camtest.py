import cv2
import time
cap = cv2.VideoCapture(0)
date = time.gmtime(time.time())
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter(str(date.tm_year)+str(date.tm_mon)+str(date.tm_mday)+'.avi', fourcc, 25.0, (640,480))

while (cap.isOpened()):
    ret, frame = cap.read()

    if ret:

        out.write(frame)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
out.release()
cv2.destroyAllWindows()
