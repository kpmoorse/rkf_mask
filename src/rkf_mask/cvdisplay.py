import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class CvDisplay(object):

    def __init__(self):

        rospy.init_node('cv_display')
        sub_topic = rospy.get_param(rospy.resolve_name("~input_image"), "/masked_image")

        self.img_sub = rospy.Subscriber(sub_topic, Image, self.callback)
        self.bridge = CvBridge()

        rospy.spin()

    def callback(self, data):

        try:
            img = self.bridge.imgmsg_to_cv2(data).copy()
        except CvBridgeError as e:
            print(e)

        cv2.imshow('image', img)
        cv2.waitKey(10)
