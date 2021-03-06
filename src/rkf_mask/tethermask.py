import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from time import time
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import scipy.interpolate as spi
# from std_msgs.msg import Int16


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

        if rospy.get_param(rospy.resolve_name("~apply_mask"), True):
            img = self.tmask(img)

        try:
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(img))
        except CvBridgeError as e:
            print(e)

    # Find and mask rectangular tether in img
    def tmask(self, img):

        # Threshold image and apply opening to mask out legs
        _, thresh = cv2.threshold(img, 120, np.iinfo(img.dtype).max, cv2.THRESH_BINARY_INV)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((15, 15)))

        # Initialize LSD with exposed parameters
        scl = rospy.get_param(rospy.resolve_name("~lsd_scale"), 0.8)
        sig = rospy.get_param(rospy.resolve_name("~lsd_sigma_scale"), 1.0)
        ang = rospy.get_param(rospy.resolve_name("~lsd_ang_th"), 25)
        lsd = cv2.createLineSegmentDetector(_refine=cv2.LSD_REFINE_NONE,
                                            _scale=scl,
                                            _sigma_scale=sig,
                                            _ang_th=ang)

        # Define ROI boundaries
        ly, lx = img.shape
        px = rospy.get_param(rospy.resolve_name("~roi_px"), 1./3)
        py = rospy.get_param(rospy.resolve_name("~roi_py"), 1./2)
        voffset = rospy.get_param(rospy.resolve_name("~roi_voffset"), 0)

        # Calculate ROI bounds
        roi_left = lx*(1-px)/2
        roi_right = lx*(1+px)/2
        roi_top = ly*voffset
        roi_bottom = ly*(voffset+py)

        lines = lsd.detect(thresh[int(roi_top):int(roi_bottom), int(roi_left):int(roi_right)])
        dy = lines[0][:, 0, 3] - lines[0][:, 0, 1]

        # Find tether polygon
        # ***Assumes tether edges are the longest straight lines in the ROI***
        edges = lines[0][np.argsort(np.abs(dy))[-2:]]
        points = edges.copy().reshape(-1, 2)
        points[:, 0] += roi_left
        points[:, 1] += roi_top

        # Truncate tether polygon for performance improvement
        trunc = rospy.get_param(rospy.resolve_name("~truncation"), 0.8)
        for i in range(2):
            sign = np.sign(points[2*i, 1] - points[2*i+1, 1])
            top = 2*i + int(sign+1)/2
            bot = 2*i + int(-sign+1)/2
            points[top, :] += trunc * (points[bot, :] - points[top, :])

        # Convert perimeter to mask and dilate
        mask = img * 0
        cv2.fillPoly(mask, [points.astype('int32')], 255)
        mask = cv2.dilate(mask, np.ones((21, 11)))

        # Inpaint over tether mask
        # rad = rospy.get_param(rospy.resolve_name("~inpaint_radius"), 8)
        img = thresh  # cv2.inpaint(thresh, mask, rad, cv2.INPAINT_TELEA)
        canny = cv2.Canny(img, 100, 200)

        mask_paint = img * 0
        pxlist = np.where(mask != 0)
        # spc = ((np.min(pxlist[0]), np.max(pxlist[0])),
        #        (np.min(pxlist[1]), np.max(pxlist[1])))

        sp_ctr = (np.max(pxlist[0]), int(np.mean(pxlist[1])))
        spy = np.arange(sp_ctr[0] - int(len(np.unique(pxlist[0]))*0.9), sp_ctr[0] + 25)
        spx = np.arange(sp_ctr[1] - 50, sp_ctr[1] + 50)
        cv2.rectangle(mask_paint, (spx[0], spy[0]), (spx[-1], spy[-1]), 255, -1)

        mask_overlap = img * 0
        canny_masked = img * 0
        cv2.bitwise_and(255 - mask, mask_paint, mask_overlap)
        cv2.bitwise_and(canny, mask_overlap, canny_masked)

        # y_proxy = np.tile(np.arange(0, img.shape[0], dtype=float)[:, None], (1, img.shape[1])).astype('uint16')

        wnd = canny[spy[0]:spy[-1], spx[0]:spx[-1]]
        submask = (self.spline_mask(wnd, range(35, 65)) * 255).astype('uint8')

        img[spy[0]:spy[-1], spx[0]:spx[-1]] = submask

        # Draw detected lines and paint mask
        draw = rospy.get_param(rospy.resolve_name("~draw_diagnostic"), False)
        if draw:
            alpha = 0.25
            img = cv2.addWeighted(mask, alpha, img, 1-alpha, 0)
            for line in lines[0]:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (int(x1 + roi_left), int(y1 + roi_top)), (int(x2 + roi_left), int(y2 + roi_top)), 255)
                cv2.circle(img, (int(x1 + roi_left), int(y1 + roi_top)), 2, 255)
                cv2.circle(img, (int(x2 + roi_left), int(y2 + roi_top)), 2, 255)

        return img

    def spline_mask(self, edge_img, ignore=None):

        # Normalize Image
        img = (edge_img - np.min(edge_img)) / np.ptp(edge_img)

        # Extract edge, excepting "ignore" points
        y_proxy = np.tile(np.arange(img.shape[0])[:, None], (1, img.shape[1])).astype('uint8')
        Y = (img * y_proxy).astype('float')
        Y[Y == 0] = np.nan
        y = np.nanmean(Y, axis=0)
        y[ignore] = np.nan

        # Apply cubic spline regression
        x = np.arange(img.shape[1])
        x2 = x.copy()
        x = x[~np.isnan(y)]
        y = y[~np.isnan(y)]
        spline = spi.UnivariateSpline(x, y)
        y2 = spline(x2)

        return y_proxy > np.tile(y2, (img.shape[0], 1))

    # *** Below functions are depricated ***

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
