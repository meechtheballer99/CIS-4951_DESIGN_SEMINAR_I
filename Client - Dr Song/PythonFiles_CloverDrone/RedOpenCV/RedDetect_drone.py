import numpy
import cv2

pos = (300,100)
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 3
color = (250,250,250)
thickness = 5

lowColorVal1 = numpy.array([170, 160, 60])
highColorVal1 = numpy.array([175, 255, 80])
lowColorVal2 = numpy.array([0, 240, 255])
highColorVal2 = numpy.array([3, 255, 255])

cam = cv2.VideoCapture(0)

if not cam.isOpened():
    print("Cannot open camera")
    exit()

cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
cv2.resizeWindow('frame',960,540)
cv2.namedWindow('mask',cv2.WINDOW_NORMAL)
cv2.resizeWindow('mask',960,540)
kernel = numpy.ones((13,13),numpy.uint8)

while True:
    # Capture frame-by-frame
    ret, frame = cam.read()

    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    final = frame.copy()
    
    mask1 = cv2.inRange(hsv, lowColorVal1, highColorVal1)
    mask2 = cv2.inRange(hsv, lowColorVal2, highColorVal2)
    mask = mask1+mask2
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    maskContours = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]
    
    result = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Display the resulting frame
    for contour in maskContours:
        x,y,w,h = cv2.boundingRect(contour)
        cv2.rectangle(final, (x,y), (x + w, y + h), (20, 255, 20), 3)
        cv2.putText(final, "Object Spotted", pos, font, fontScale, color, thickness, cv2.LINE_AA)
    
    cv2.imshow('frame', final)
    cv2.imshow('mask', result)
    
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the captureq
cam.release()
cv2.destroyAllWindows()
