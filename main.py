import cv2
import numpy as np

# Open the camera
cap = cv2.VideoCapture(0)

# Check if the camera is opened correctly
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # if frame is read correctly ret is True
    if not ret:
        sleep(0.25)
        continue  

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the range of bright orange to bright red color in HSV
    lower_color = np.array([0, 180, 180])
    upper_color = np.array([255, 255, 255])

    # Threshold the HSV image to get only bright orange to bright red colors
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Find contours in the mask
    # For this problem, finding the largest contour is a simpler and more direct approach
    # than implementing a full Euclidean clustering algorithm.
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Set a minimum contour area to filter out small objects
    min_contour_area = 500  # Adjust this value as needed

    if contours:
        # Filter contours by area and find the largest one
        large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]
        
        if large_contours:
            largest_contour = max(large_contours, key=cv2.contourArea)
            
            # Calculate moments for the largest contour
            M = cv2.moments(largest_contour)
            
            # Calculate x,y coordinate of center
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                # Draw a circle at the center
                cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
                # Display the coordinates on the frame
                cv2.putText(frame, f"({cX}, {cY})", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Bitwise-AND mask and original image if a large contour is found
            res = cv2.bitwise_and(frame, frame, mask=mask)
        else:
            # If no large contour is found, display the original frame
            res = frame
    else:
        # If no contours are found at all, display the original frame
        res = frame
    
    # Display the resulting frame
    cv2.imshow('Original Frame', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Detected Color', res)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
