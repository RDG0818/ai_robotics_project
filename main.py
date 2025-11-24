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
        rospy.logerr("Cannot read from camera")
        cap.release()
        return
    frame_height, frame_width, _ = frame.shape
    center_x = frame_width // 2
    center_tolerance = 50 

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if not ret or frame is None:
            rospy.logwarn("Failed to grab frame")
            sleep(0.25)
            continue  

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Orange color detection range
        lower_color = np.array([5, 180, 180])
        upper_color = np.array([255, 255, 255])

        mask = cv2.inRange(hsv, lower_color, upper_color)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        min_contour_area = 500
        
        if contours:
            large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]
            
            if large_contours:
                largest_contour = max(large_contours, key=cv2.contourArea)
                
                M = cv2.moments(largest_contour)
                
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    
                    # Centering logic
                    if cX < center_x - center_tolerance:
                        # Turn right
                        bot.move(0.0, -0.3)
                    elif cX > center_x + center_tolerance:
                        # Turn left
                        bot.move(0.0, 0.3)
                    else:
                        # Centered, now check size
                        contour_area = cv2.contourArea(largest_contour)
                        sleep(0.5) 
                        if (contour_area / (frame_width * frame_height)) < 0.5:
                            # Move forward
                            rospy.loginfo("Object centered, moving forward.")
                            bot.move(0.2, 0.0)
                        else:
                            # Stop and sleep for a second
                            rospy.loginfo("Object is large enough. Stopping and sleeping.")
                            bot.move(0.0, 0.0)
                            sleep(0.5)
                else:
                    # case where m00 is 0 but contour exists
                    bot.move(0.0, 0.0)
            else:
                bot.move(0.0, 0.0) # No large contour, stop
        else:
            bot.move(0.0, 0.0) # No contours, stop
        
        rate.sleep()

    # Stop the robot and release resources
    bot.move(0.0, 0.0)
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
