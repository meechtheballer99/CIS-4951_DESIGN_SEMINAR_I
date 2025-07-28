import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('red_object_detection')
bridge = CvBridge()

# Create a publisher to publish the video feed to a ROS topic
image_pub = rospy.Publisher('red_object_detection/image_raw', Image, queue_size=10)

def detect_red(cv_image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define the range of red color in HSV
    lower_red = (0, 50, 50)
    upper_red = (10, 255, 255)
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = (170, 50, 50)
    upper_red = (180, 255, 255)
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    # Combine the masks to obtain the final mask
    mask = mask1 + mask2

    # Apply the mask to the original image
    res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    return res


def image_callback(data):
    # Convert the ROS image message to an OpenCV image
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')

    # Detect the red objects in the image
    red_image = detect_red(cv_image)

    # Convert the OpenCV image back to a ROS image message
    img_msg = bridge.cv2_to_imgmsg(red_image, encoding="bgr8")

    # Publish the ROS image message to the topic
    image_pub.publish(img_msg)

# Create a subscriber to receive video frames from the camera
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

rospy.spin()
