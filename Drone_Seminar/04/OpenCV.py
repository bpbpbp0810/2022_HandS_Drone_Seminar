import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# Print the size of frames
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(width, height)

while True:
    # Load a frame
    _, frame = cap.read()

    cvx = None
    cvy = None

    # Convert the color space from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # HSV range for red ~ orange
    l_b = np.array([0, 150, 150])
    u_b = np.array([25, 360, 360])

    # Extract a binary image
    mask = cv2.inRange(hsv, l_b, u_b)
    blur = cv2.GaussianBlur(mask, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 10, 255, cv2.THRESH_BINARY)
    dilated = cv2.dilate(thresh, None, iterations=3)
    contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw a contour box around the largest detection
    if len(contours) != 0: 
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 1000:
            (x, y, w, h) = cv2.boundingRect(largest)        
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cvx = int((x+w/2)-320)
            cvy = -1 * int((y+h/2)-240)
            cv2.putText(frame, "Area : " + str(w*h), (x-10, y-25), cv2.FONT_ITALIC, 0.5, (255, 20, 20), 2)
            cv2.putText(frame, "Distance_from_center : (" + str(cvx) + "," +str(cvy) + ")", (x-10, y-10), cv2.FONT_ITALIC, 0.5, (255, 20, 20), 2)

    cv2.rectangle(frame, (int(width / 2) - 50, int(height / 2) - 50), (int(width / 2) + 50, int(height / 2) + 50), (255, 0, 0), 3)
    cv2.line(frame, (int(width / 2) - 10, int(height / 2)), (int(width / 2) + 10, int(height / 2)), (0, 0, 255), 3)
    cv2.line(frame, (int(width / 2), int(height / 2) - 10), (int(width / 2), int(height / 2) + 10), (0, 0, 255), 3)
    cv2.imshow("final", frame)            

    if cv2.waitKey(100) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()