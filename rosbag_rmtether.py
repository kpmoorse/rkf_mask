import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
import imutils
import numpy as np


def rosbag_rmtether(fin, pause=False):

    bag = rosbag.Bag(fin, "r")
    bridge = CvBridge()

    for _, msg, _ in tqdm(bag, total=bag.get_message_count()):

        img = bridge.imgmsg_to_cv2(msg).copy()

        # Extract tether polygon
        lsd = cv2.createLineSegmentDetector()
        ly, lx = img.shape
        lines = lsd.detect(img[:int(ly/2), int(lx/3):int(2*lx/3)])
        d = np.sqrt((lines[0][:, 0, 3] - lines[0][:, 0, 1])**2 + (lines[0][:,0,3] - lines[0][:,0,1])**2)
        edges = lines[0][np.argsort(d)[-2:]]
        points = edges.copy().reshape(-1, 2)
        points[:, 0] += lx/3

        mask = img * 0
        cv2.fillPoly(mask, [points.astype('int32')], 255)
        mask = cv2.dilate(mask, np.ones((3, 11)))

        # cm = np.mean(points, axis=0)
        # points[:, 0] += 0*(points[:, 0] - cm[0])

        # Find statistical mode
        hist = np.histogram(img, 256)
        mode = hist[1][np.argsort(hist[0])[-2]]

        # c =
        # cv2.fillPoly(img, [points.astype('int32')], mode)
        # img = cv2.bitwise_and(img, img*0, mask=mask)
        img[mask == 255] = mode
        cv2.imshow('image', img)

        if pause:
            cv2.waitKey(0)
        else:
            cv2.waitKey(20)

    cv2.destroyAllWindows()


# Restrict values to (-pi, pi]
def pi_mod(theta):
    return (theta + np.pi) % (2*np.pi) - np.pi


rosbag_rmtether('rkf_test01.bag', pause=False)
