import numpy as np
import cv2
from matplotlib import pyplot as plt
img = cv2.imread('glossy/rendered_glossyusenew_4000.png')

clean = cv2.fastNlMeansDenoising(img, 2.0, 4, 21)
cv2.imwrite("filter_fixed_diffuse.png", clean)
# create a list of first 5 frames
# img = [cap.read()[1] for i in xrange(5)]
# convert all to grayscale
# gray = [cv.cvtColor(i, cv.COLOR_BGR2GRAY) for i in img]
# # convert all to float64
# gray = [np.float64(i) for i in gray]
# # create a noise of variance 25
# noise = np.random.randn(*gray[1].shape)*10
# # Add this noise to images
# noisy = [i+noise for i in gray]
# # Convert back to uint8
# noisy = [np.uint8(np.clip(i,0,255)) for i in noisy]
# # Denoise 3rd frame considering all the 5 frames
# dst = cv.fastNlMeansDenoisingMulti(noisy, 2, 5, None, 4, 7, 35)
# plt.subplot(131),plt.imshow(gray[2],'gray')
# plt.subplot(132),plt.imshow(noisy[2],'gray')
# plt.subplot(133),plt.imshow(dst,'gray')
# plt.show()