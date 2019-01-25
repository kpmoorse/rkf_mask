import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from time import time
from sensor_msgs.msg import Image


class TetherMask(object):

    def __init__(self):

        rospy.init_node('tether_mask')
        sub_topic = rospy.get_param(rospy.resolve_name("~input_image"), "stabilized_image")
        pub_topic = rospy.get_param(rospy.resolve_name("~output_image"), "masked_image")

        self.img_pub = rospy.Publisher(pub_topic, Image, queue_size=10)
        self.img_sub = rospy.Subscriber(sub_topic, Image, self.callback)
        self.bridge = CvBridge()

        rospy.spin()

    # When message is received, apply mask and publish
    def callback(self, data):

        try:
            img = self.bridge.imgmsg_to_cv2(data).copy()
        except CvBridgeError as e:
            print(e)

        img = self.tmask(img)

        try:
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(img))
        except CvBridgeError as e:
            print(e)

    # Find and mask rectangular tether in img
    def tmask(self, img):

        # Find tether polygon
        # ***Assumes tether edges are the longest straight lines in the ROI***
        lsd = cv2.createLineSegmentDetector(_refine=cv2.LSD_REFINE_NONE)
        ly, lx = img.shape
        # ROI is the top-middle segment of a 2x3 grid
        lines = lsd.detect(img[:int(ly / 2), int(lx / 3):int(2 * lx / 3)])
        d = np.sqrt((lines[0][:, 0, 3] - lines[0][:, 0, 1]) ** 2 + (lines[0][:, 0, 3] - lines[0][:, 0, 1]) ** 2)
        edges = lines[0][np.argsort(d)[-2:]]
        points = edges.copy().reshape(-1, 2)
        points[:, 0] += lx / 3

        # Truncate tether polygon
        trunc = rospy.get_param(rospy.resolve_name("~truncation"), 0.5)
        for i in range(2):
            sign = np.sign(points[2*i, 1] - points[2*i+1, 1])
            top = 2*i + int(sign+1)/2
            bot = 2*i + int(-sign+1)/2
            points[top, :] += trunc * (points[bot, :] - points[top, :])

        mask = img * 0
        cv2.fillPoly(mask, [points.astype('int32')], 255)
        mask = cv2.dilate(mask, np.ones((21, 15)))

        # Inpaint over tether mask
        img = cv2.inpaint(img, mask, 10, cv2.INPAINT_TELEA)

        if rospy.get_param(rospy.resolve_name("~draw_diagnostic"), False):
            for line in lines[0]:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (int(x1+lx/3), y1), (int(x2+lx/3), y2), (255, 255, 255))

        return img

    # *** Below functions are for testing only ***

    def msg_to_img(self, msg):

        imgmsg = msg[1]
        img = self.bridge.imgmsg_to_cv2(imgmsg).copy()
        return img

    def masked_playback(self, bag, framelock=30):

        for msg in bag:

            if framelock > 0:
                tframe = time()
            img = self.msg_to_img(msg)
            img = self.tmask(img)

            cv2.imshow('image', img)
            if framelock > 0:
                cv2.waitKey(max(1, int((1. / framelock - (time() - tframe)) * 1000)))
            else:
                cv2.waitKey(10)
