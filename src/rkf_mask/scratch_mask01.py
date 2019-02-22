import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as spi
import statsmodels.nonparametric.smoothers_lowess as sm


def spline_mask(edge_img, ignore=None):
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


img = cv2.imread('/home/dickinsonlab/git/rkf_analysis/example_plots/test01.png', cv2.IMREAD_GRAYSCALE)
mask = spline_mask(img, range(35, 65))

plt.figure()
plt.imshow(mask)

# print(np.unique(img))