import cv2
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Create a folder named "images" if it doesn't exist
images_folder = "images"
if not os.path.exists(images_folder):
    os.makedirs(images_folder)

# Callback function to receive images from the topic
def image_callback(msg):
    try:
        # Convert ROS image message to OpenCV format
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('frame', cv_image)

        # Save images when 's' is pressed
        if cv2.waitKey(1) & 0xFF == ord('s'):
            image_name = os.path.join(images_folder, f"image_{cv2.getTickCount()}.png")
            cv2.imwrite(image_name, cv_image)
            print(f"Image saved: {image_name}")

    except Exception as e:
        print(e)

# Initialize the node
rospy.init_node('image_subscriber')

# Subscribe to the image topic
image_topic = "/your/image/topic"  # Replace this with the actual image topic
rospy.Subscriber(image_topic, Image, image_callback)

# Keep the program running until 'q' is pressed
while not rospy.is_shutdown():
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Close OpenCV windows
cv2.destroyAllWindows()
