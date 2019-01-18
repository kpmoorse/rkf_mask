import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
import imutils
import numpy as np


def rosbag_rmtether(fin, pause=False, bgcalc=True):

    bag = rosbag.Bag(fin, "r")
    bridge = CvBridge()

    for _, msg, _ in tqdm(bag, total=bag.get_message_count()):

        img = bridge.imgmsg_to_cv2(msg).copy()

        # Extract tether polygon
        lsd = cv2.createLineSegmentDetector()
        ly, lx = img.shape
        lines = lsd.detect(img[:int(ly/2), int(lx/3):int(2*lx/3)])
        d = np.sqrt((lines[0][:, 0, 3] - lines[0][:, 0, 1])**2 + (lines[0][:, 0, 3] - lines[0][:, 0, 1])**2)
        edges = lines[0][np.argsort(d)[-2:]]
        points = edges.copy().reshape(-1, 2)
        points[:, 0] += lx/3

        nzx = sum(img[-1, :] != 0)
        nzy = sum(img[:, 1] != 0)
        img_tmp = img[-nzy:, :nzx]

        mask = img_tmp * 0
        cv2.fillPoly(mask, [points.astype('int32')], 255)
        mask = cv2.dilate(mask, np.ones((21, 13)))

        img[-nzy:, :nzx] = cv2.inpaint(img[:nzy, :nzx], mask, 10, cv2.INPAINT_TELEA)
        cv2.imshow('image', img)

        if pause:
            cv2.waitKey(0)
        else:
            cv2.waitKey(20)

    cv2.destroyAllWindows()


# Restrict values to (-pi, pi]
def pi_mod(theta):
    return (theta + np.pi) % (2*np.pi) - np.pi


rosbag_rmtether('rkf_test01.bag', pause=False, bgcalc=False)
