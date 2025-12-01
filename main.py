import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from time import sleep

class TwistPublisher():
    def __init__(self):
        rospy.init_node("twist_pub")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def move(self, cmd_x, cmd_z):
        msg = Twist()
        msg.linear.x = cmd_x
        msg.angular.z = cmd_z
        self.pub.publish(msg)
        rospy.loginfo("Published Twist: linear.x=%f angular.z=%f", cmd_x, cmd_z)

def main():
    DEVICE_INDEX = 0
    cap = cv2.VideoCapture(DEVICE_INDEX)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        return

    bot = TwistPublisher()
    rospy.sleep(2.0) # Give time for publisher to connect

    # Get frame dimensions
    ret, frame = cap.read()
    if not ret:
        rospy.loger("Cannot read from camera")
        cap.release()
        return
    frame_height, frame_width, _ = frame.shape
    center_x = frame_width // 2
    center_tolerance = 120 
    turning = False

    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            cv2.imwrite("frame.jpg", frame)

        if not ret or frame is None:
            rospy.logwarn("Failed to grab frame")
            rospy.sleep(0.1)
            continue  

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Orange color detection range
        lower_color = np.array([0, 80, 100])
        upper_color = np.array([50, 255, 255])

        mask = cv2.inRange(hsv, lower_color, upper_color)
        cv2.imwrite("frame_masked.jpg", mask)
        print("masked image")
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        min_contour_area = 500
        print(turning)
        large_contours = []
        if contours: 
            print("saw a contour")
            large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]
        if (not turning) and (not large_contours):
            bot.move(0.0, 0.3)
            print("searching for orange")
            if large_contours:
                print("saw a large contour")
                turning = True
                bot.move(0.0, 0.0)
        
        if contours:
            print("checking contours")
            large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]
            
            if large_contours:
                largest_contour = max(large_contours, key=cv2.contourArea)
                
                M = cv2.moments(largest_contour)
                
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    
                    # Centering logic
                    if cX < center_x - center_tolerance:
                        # Turn right
                        bot.move(0.0, 0.3)
                    elif cX > center_x + center_tolerance:
                        # Turn left
                        bot.move(0.0, -0.3)
                    else:
                        # Centered, now check size
                        contour_area = cv2.contourArea(largest_contour)
                        orange_percent = (contour_area / (frame_width * frame_height))
                        print(orange_percent)
                        if (contour_area / (frame_width * frame_height)) < 0.07:
                            # Move forward
                            rospy.loginfo("Object centered, moving forward.")
                            bot.move(0.4, 0.0)
                        else:
                            # Stop and sleep for a second
                            rospy.loginfo("Object is large enough. Stopping and sleeping.")
                            bot.move(0.0, 0.0)
                            rospy.sleep(0.5)
                else:
                    # case where m00 is 0 but contour exists
                    print("m00 case stop")
                    bot.move(0.0, 0.0)
        
        rate.sleep()

    # Stop the robot and release resources
    bot.move(0.0, 0.0)
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

