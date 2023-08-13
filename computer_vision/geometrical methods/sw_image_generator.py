#!/usr/bin/env python
from std_msgs.msg import Int32
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
global received_angle
received_angle=0
def rotation_callback(msg):
    global received_angle
    received_angle = msg.data
    rospy.loginfo("Received integer: %d", received_angle)

def rotate_image(image, angle):
    height, width = image.shape[:2]
    center = (width // 2, height // 2)
    rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height))
    return rotated_image

def image_callback(msg):
    global received_angle
    angle=received_angle
    print("in")
    bridge = CvBridge()
    im = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    

    # Get the height and width of the image
    height, width, _ = im.shape
    rospy.loginfo("Image dimensions - Height: %d, Width: %d", height, width)

    #rotated_image = cv2.rotate(im, cv2.ROTATE_90_CLOCKWISE)
    rotated_image = rotate_image(im, angle)
    compression_params = [cv2.IMWRITE_JPEG_QUALITY, 90]
    _, compressed_image = cv2.imencode('.jpg', rotated_image, compression_params)
    compressed_data = compressed_image.tobytes()

    # Create a CompressedImage message
    compressed_msg = CompressedImage()
    compressed_msg.header = msg.header
    compressed_msg.format = "jpeg"
    compressed_msg.data = compressed_data

    # Publish the compressed image
    pub.publish(compressed_msg)


def image_subscriber():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber('/zed2i/zed_node/left/image_rect_color/compressed', CompressedImage, image_callback)
    rospy.spin()

if __name__ == '__main__':
    print("hello")
    pub = rospy.Publisher('rotated_image_topic_c', CompressedImage, queue_size=10)
    rospy.Subscriber('angle_image', Int32, rotation_callback)
    image_subscriber()