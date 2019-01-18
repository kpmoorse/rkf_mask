import rosbag
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm
import imutils
import numpy as np


def rosbag_exshape(fin, pause=False):

    bag = rosbag.Bag(fin, "r")
    bridge = CvBridge()

    for _, msg, _ in tqdm(bag, total=bag.get_message_count()):

        img = bridge.imgmsg_to_cv2(msg)

        # Extract and sharpen stabilization border
        lower = 0
        upper = 1
        edgeMask = cv2.inRange(img, lower, upper)
        edgeMask = cv2.dilate(edgeMask, np.ones((3, 3)))
        img = cv2.bitwise_and(img, img, mask=255-edgeMask)

        # Extract foreground (dark)
        lower = 1
        upper = 100
        shapeMask = cv2.inRange(img, lower, upper)

        # Extract contours from foreground
        contours = cv2.findContours(shapeMask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        contour = contours[np.argmax([len(contour) for contour in contours])]
        # contour = contour[::2, :]

        lines = cv2.HoughLinesP(shapeMask, 20, np.pi / 180, 5000)

        lsd = cv2.createLineSegmentDetector()
        ly, lx = img.shape
        lines = lsd.detect(img[:int(ly/2), int(lx/3):int(2*lx/3)])
        d = np.sqrt((lines[0][:, 0, 3] - lines[0][:, 0, 1])**2 + (lines[0][:,0,3] - lines[0][:,0,1])**2)
        edges = lines[0][np.argsort(d)[-2:]]
        corners = edges.copy().reshape(-1, 2)
        corners[:, 0] += lx/3

        # cr = contour.reshape(contour.shape[0], -1)
        # crdiff = np.diff(cr, axis=0)
        # # crlong = cr[1:][np.linalg.norm(crdiff, axis=1) > 10]
        # angle = np.arctan2(crdiff[:, 0], crdiff[:, 1])
        # delta_angle = pi_mod(np.diff(angle))
        # straights = np.abs(np.abs(delta_angle)) < 0.1
        # straights = cr[1:-1][straights]

        # Display contours
        cv2.imshow('image', shapeMask)

        # for line in edges:
        #     for x1, y1, x2, y2 in line:
        #         cv2.line(img, (int(x1+lx/3), y1), (int(x2+lx/3), y2), (255, 255, 255), 2)

        # c =
        cv2.fillPoly(img, [corners.astype('int32')], (255, 255, 255))

        # cv2.fillPoly(img)

        # cv2.drawContours(img, [cr[::2].copy()], -1, 255, 2)
        # for point in straights:
        #     cv2.circle(img, tuple(point), 2, 255)

        # lsd.drawSegments(img, lines[0])
        cv2.imshow('image', img)

        if pause:
            cv2.waitKey(0)
        else:
            cv2.waitKey(20)

    cv2.destroyAllWindows()


# Restrict values to (-pi, pi]
def pi_mod(theta):
    return (theta + np.pi) % (2*np.pi) - np.pi


rosbag_exshape('rkf_test01.bag', pause=False)
